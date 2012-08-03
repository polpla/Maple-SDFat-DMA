#include "Sd2Card.h"

#define DO_DMA_WRITE
bool dmaActive;
uint8_t mysink[1];

inline void DMAEvent(){
    dma_irq_cause event = dma_get_irq_cause(DMA1, DMA_CH3);
    switch(event) {
        case DMA_TRANSFER_COMPLETE:
            dma_detach_interrupt(DMA1, DMA_CH2);
            dma_detach_interrupt(DMA1, DMA_CH3);
            dmaActive = false;
            break;
        case DMA_TRANSFER_ERROR:
            SerialUSB.println("DMA Error - read/write data might be corrupted");
            break;
    }
}


Sd2Card::Sd2Card() : errorCode_(0), inBlock_(0), partialBlockRead_(0), type_(0), HardwareSPI(1) {
    this->begin(SPI_18MHZ, MSBFIRST, 0);
    //init DMA
    dma_init(DMA1);
    //enable SPI over DMA
    spi_rx_dma_enable(SPI1);
    spi_tx_dma_enable(SPI1);
    //DMA activity control
    dmaActive = false;
    //Acknowledgment array
    for(int i=0; i<SPI_BUFF_SIZE; i++) 
        ack[i] = 0xFF;
}

void Sd2Card::spiSend(uint8_t b) {
	this->transfer(b);   // thd write to transfer
//	this->write(b);  
}

uint8_t Sd2Card::spiRec(void)  {
	return this->transfer(0XFF);
}

uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
    readEnd();
    CS0;
    waitNotBusy(300);
    
    spiSend(cmd );

    spiSend(arg >> 24);
    spiSend(arg >> 16);
    spiSend(arg >> 8);
    spiSend(arg >> 0);

    uint8_t crc = 0XFF;
    if (cmd == CMD0) crc = 0X95;  // correct crc for CMD0 with arg 0
    if (cmd == CMD8) crc = 0X87;  // correct crc for CMD8 with arg 0X1AA
    spiSend(crc);

    for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++);
    return status_;
}


uint32_t Sd2Card::cardSize(void) {
    csd_t csd;
    if (!readCSD(&csd)) return 0;
    if (csd.v1.csd_ver == 0) {
        uint8_t read_bl_len = csd.v1.read_bl_len;
        uint16_t c_size = (csd.v1.c_size_high << 10) | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
        uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1) | csd.v1.c_size_mult_low;
        return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
    } else if (csd.v2.csd_ver == 1) {
        uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16) | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
        return (c_size + 1) << 10;
    } else {
        error(SD_CARD_ERROR_BAD_CSD);
        return 0;
    }
}

uint8_t Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock) {
    if (!eraseSingleBlockEnable()) {
        error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
        SerialUSB.println("Error: Erase Single Block");
        goto fail;
    }
    if (type_ != SD_CARD_TYPE_SDHC) {
        firstBlock <<= 9;
        lastBlock <<= 9;
    }
    if (cardCommand(CMD32, firstBlock) || cardCommand(CMD33, lastBlock) || cardCommand(CMD38, 0)) {
        error(SD_CARD_ERROR_ERASE);
        SerialUSB.println("Error: Erase");
        goto fail;
    }
    if (!waitNotBusy(SD_ERASE_TIMEOUT)) {
        error(SD_CARD_ERROR_ERASE_TIMEOUT);
        SerialUSB.println("Error: Erase timeout");
        goto fail;
    }
    CS1;
    return true;

fail:
    CS1;
    SerialUSB.println("Error: Sd2Card::Erase()");
    return false;
}


uint8_t Sd2Card::eraseSingleBlockEnable(void) {
    csd_t csd;
    return readCSD(&csd) ? csd.v1.erase_blk_en : 0;
}


uint8_t Sd2Card::init() {
    errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;
    uint16_t t0 = (uint16_t)millis();
    uint32_t arg;

    gpio_set_mode(CSPORT, CS, GPIO_OUTPUT_PP);
    
    CS1;
    for (uint8_t i = 0; i < 10; i++) 
        spiSend(0XFF);
    CS0;

    // command to go idle in SPI mode
    while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
        if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
            SerialUSB.println("Error: CMD0");
            error(SD_CARD_ERROR_CMD0);
            goto fail;
        }
    }
    // check SD version
    if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
        type(SD_CARD_TYPE_SD1);
    } 
    else {
    // only need last byte of r7 response
    for (uint8_t i = 0; i < 4; i++) 
		status_ = spiRec();
        if (status_ != 0XAA) {
            error(SD_CARD_ERROR_CMD8);
            SerialUSB.println("Error: CMD8");
            goto fail;
        }
        type(SD_CARD_TYPE_SD2);
    }
    // initialize card and send host supports SDHC if SD2
    arg = (type() == SD_CARD_TYPE_SD2) ? 0X40000000 : 0;

    while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
    // check for timeout
        if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
            SerialUSB.println("Error: ACMD41");
            error(SD_CARD_ERROR_ACMD41);
            goto fail;
        }
    }
    // if SD2 read OCR register to check for SDHC card
    if (type() == SD_CARD_TYPE_SD2) {
        if (cardCommand(CMD58, 0)) {
            SerialUSB.println("Error: CMD58");
            error(SD_CARD_ERROR_CMD58);
            goto fail;
        }
        if ((spiRec() & 0XC0) == 0XC0) 
            type(SD_CARD_TYPE_SDHC);
        // discard rest of ocr - contains allowed voltage range
        for (uint8_t i = 0; i < 3; i++) 
            spiRec();
    }
    CS1;

    return true;

fail:
    CS1;
    SerialUSB.println("Error: Sd2Card::init()");
    return false;
}

void Sd2Card::partialBlockRead(uint8_t value) {
    readEnd();
    partialBlockRead_ = value;
}

uint8_t Sd2Card::readBlock(uint32_t block, uint8_t* dst) {
    return readData(block, 0, 512, dst);
}

uint8_t Sd2Card::readData(uint32_t block, uint16_t offset, uint16_t count, uint8_t* dst) {
    
    uint16_t n;
    
    if (count == 0) return true;
    if ((count + offset) > 512) goto fail;
    
    if (!inBlock_ || block != block_ || offset < offset_) {
        block_ = block;
        // use address if not SDHC card
        if (type() != SD_CARD_TYPE_SDHC) 
            block <<= 9;
        if (cardCommand(CMD17, block)) {
            error(SD_CARD_ERROR_CMD17);
            SerialUSB.println("Error: CMD17");
            goto fail;
        }
        if (!waitStartBlock()) {
            goto fail;
        }
        offset_ = 0;
        inBlock_ = 1;
    }
    // skip data before offset
    if(offset_ < offset){
        dma_setup_transfer(DMA1, DMA_CH3, &SPI1->regs->DR, DMA_SIZE_8BITS, ack, DMA_SIZE_8BITS,
                           (/*DMA_MINC_MODE | DMA_CIRC_MODE  |*/ DMA_FROM_MEM | DMA_TRNS_CMPLT | DMA_TRNS_ERR));
        dma_attach_interrupt(DMA1, DMA_CH3, DMAEvent);
        dma_set_priority(DMA1, DMA_CH3, DMA_PRIORITY_VERY_HIGH);
        dma_set_num_transfers(DMA1, DMA_CH3, offset - offset_);
        
        dmaActive = true;
        dma_enable(DMA1, DMA_CH3);
        
        while(dmaActive) delayMicroseconds(1);
        dma_disable(DMA1, DMA_CH3);
    }
    offset_ = offset;
    
    // transfer data
    dma_setup_transfer(DMA1, DMA_CH2, &SPI1->regs->DR, DMA_SIZE_8BITS, dst, DMA_SIZE_8BITS,
                       (DMA_MINC_MODE | DMA_TRNS_CMPLT | DMA_TRNS_ERR));
    dma_attach_interrupt(DMA1, DMA_CH2, DMAEvent);
    dma_setup_transfer(DMA1, DMA_CH3, &SPI1->regs->DR, DMA_SIZE_8BITS, ack, DMA_SIZE_8BITS,
                       (/*DMA_MINC_MODE | DMA_CIRC_MODE |*/ DMA_FROM_MEM));             
    dma_set_priority(DMA1, DMA_CH2, DMA_PRIORITY_VERY_HIGH);
    dma_set_priority(DMA1, DMA_CH3, DMA_PRIORITY_VERY_HIGH);
    dma_set_num_transfers(DMA1, DMA_CH2, count);
    dma_set_num_transfers(DMA1, DMA_CH3, count);
    
    dmaActive = true;
    dma_enable(DMA1, DMA_CH3);
    dma_enable(DMA1, DMA_CH2);
    
    while(dmaActive) delayMicroseconds(1);
    dma_disable(DMA1, DMA_CH3);
    dma_disable(DMA1, DMA_CH2);
    
    offset_ += count;
    if (!partialBlockRead_ || offset_ >= SPI_BUFF_SIZE) {
        readEnd();
    }
    return true;

fail:
    CS1;
    SerialUSB.println("Error: Sd2Card::readData()");
    return false;
}

void Sd2Card::readEnd(void) {
    if (inBlock_) {
        dma_setup_transfer(DMA1, DMA_CH3, &SPI1->regs->DR, DMA_SIZE_8BITS, ack, DMA_SIZE_8BITS,
                           (/*DMA_MINC_MODE | DMA_CIRC_MODE |*/ DMA_FROM_MEM | DMA_TRNS_CMPLT | DMA_TRNS_ERR));  
        dma_attach_interrupt(DMA1, DMA_CH3, DMAEvent);
        dma_set_priority(DMA1, DMA_CH3, DMA_PRIORITY_VERY_HIGH);
        dma_set_num_transfers(DMA1, DMA_CH3, SPI_BUFF_SIZE + 1 - offset_);
        
        dmaActive = true;
        dma_enable(DMA1, DMA_CH3);
        
        while(dmaActive)delayMicroseconds(1);
        dma_disable(DMA1, DMA_CH3);
        
        CS1;
        inBlock_ = 0;
    }
}

uint8_t Sd2Card::readRegister(uint8_t cmd, void* buf) {
    uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
    if (cardCommand(cmd, 0)) {
        error(SD_CARD_ERROR_READ_REG);
        SerialUSB.println("Error: Read reg");
        goto fail;
    }
    if (!waitStartBlock()) 
        goto fail;
    // transfer data
    for (uint16_t i = 0; i < 16; i++) 
        dst[i] = spiRec();
    spiRec();  // get first crc byte
    spiRec();  // get second crc byte
    CS1;
    return true;

fail:
    SerialUSB.println("Error: Sd2Card::readRegister()");
    CS1;
    return false;
}

uint8_t Sd2Card::waitNotBusy(uint16_t timeoutMillis) {
    uint16_t t0 = millis();
    do {
        if (spiRec() == 0XFF) 
		return true;
    }
    while (((uint16_t)millis() - t0) < timeoutMillis);
    return false;
}

uint8_t Sd2Card::waitStartBlock(void) {
    uint16_t t0 = millis();
    while ((status_ = spiRec()) == 0XFF) {
        if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT) {
            error(SD_CARD_ERROR_READ_TIMEOUT);
            SerialUSB.println("Error: Read timeout");
            goto fail;
        }
    }
    if (status_ != DATA_START_BLOCK) {
        SerialUSB.print(DATA_START_BLOCK);
        SerialUSB.print(" != ");
        SerialUSB.println(status_);
        error(SD_CARD_ERROR_READ);
        SerialUSB.println("Error: Read");
        goto fail;
    }
    return true;

fail:
    CS1;
    SerialUSB.println("Error: Sd2Card::waitStartBlock()");
    return false;
}

uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src) {
#if SD_PROTECT_BLOCK_ZERO
    // don't allow write to first block
    if (blockNumber == 0) {
        error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
        SerialUSB.println("Error: Write block zero");
        goto fail;
    }
#endif  // SD_PROTECT_BLOCK_ZERO

    // use address if not SDHC card
    if (type() != SD_CARD_TYPE_SDHC) 
        blockNumber <<= 9;
    if (cardCommand(CMD24, blockNumber)) {
        SerialUSB.println("Error: CMD24");
        error(SD_CARD_ERROR_CMD24);
        goto fail;
    }
    if (!writeData(DATA_START_BLOCK, src)) 
        goto fail;

    // wait for flash programming to complete
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
        error(SD_CARD_ERROR_WRITE_TIMEOUT);
        SerialUSB.println("Error: Write timeout");
        goto fail;
    }
    // response is r2 so get and check two bytes for nonzero
    if (cardCommand(CMD13, 0) || spiRec()) {
        error(SD_CARD_ERROR_WRITE_PROGRAMMING);
        SerialUSB.println("Error: Write programming");
        goto fail;
    }
    CS1;
    return true;

fail:
    CS1;
    SerialUSB.println("Error: Sd2Card::writeBlock");
    return false;
}

uint8_t Sd2Card::writeData(const uint8_t* src) {
    // wait for previous write to finish
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
        error(SD_CARD_ERROR_WRITE_MULTIPLE);
        SerialUSB.println("Error: writeData");
        CS1;
        return false;
    }
    return writeData(WRITE_MULTIPLE_TOKEN, src);
}

uint8_t Sd2Card::writeData(uint8_t token, const uint8_t* src) {
    spiSend(token);
#ifdef DO_DMA_WRITE
        dma_setup_transfer(DMA1, DMA_CH3, &SPI1->regs->DR, DMA_SIZE_8BITS, (uint8_t *)src, DMA_SIZE_8BITS, (DMA_MINC_MODE |  DMA_FROM_MEM | DMA_TRNS_CMPLT | DMA_TRNS_ERR));
        dma_attach_interrupt(DMA1, DMA_CH3, DMAEvent);
        dma_set_priority(DMA1, DMA_CH3, DMA_PRIORITY_VERY_HIGH);
        dma_set_num_transfers(DMA1, DMA_CH3, 512);

        dmaActive = true;
        dma_enable(DMA1, DMA_CH3);

        while(dmaActive) delayMicroseconds(1);
        dma_disable(DMA1, DMA_CH3);
#else
    for (uint16_t i = 0; i < 512; i++) {
        spiSend(src[i]);
    }
#endif
    spiSend(0xff);  // dummy crc
    spiSend(0xff);  // dummy crc

    while((status_ = spiRec()) == 0xff);  // thd catch up hack
    if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
        error(SD_CARD_ERROR_WRITE);
        CS1;
		SerialUSB.print(status_,HEX);
        SerialUSB.println(" Error: Write");
        SerialUSB.println("Error: Sd2Card::writeData()");
        return false;
    }
    return true;
}

uint8_t Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount) {
#if SD_PROTECT_BLOCK_ZERO
    // don't allow write to first block
    if (blockNumber == 0) {
        error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
        SerialUSB.println("Error: Write block zero");
        goto fail;
    }
#endif  // SD_PROTECT_BLOCK_ZERO
    // send pre-erase count
    if (cardAcmd(ACMD23, eraseCount)) {
        SerialUSB.println("Error: ACMD23");
        error(SD_CARD_ERROR_ACMD23);
        goto fail;
    }
    // use address if not SDHC card
    if (type() != SD_CARD_TYPE_SDHC) 
        blockNumber <<= 9;
    if (cardCommand(CMD25, blockNumber)) {
        error(SD_CARD_ERROR_CMD25);
        SerialUSB.println("Error: CMD25");
        goto fail;
    }
    return true;

fail:
    CS1;
    SerialUSB.println("Error: Sd2Card::writeStart()");
    return false;
}

uint8_t Sd2Card::writeStop(void) {
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) 
        goto fail;
    spiSend(STOP_TRAN_TOKEN);
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) 
        goto fail;
    CS1;
    return true;

fail:
    error(SD_CARD_ERROR_STOP_TRAN);
    CS1;
    SerialUSB.println("Error: Sd2Card::writeStop()");
    return false;
}
