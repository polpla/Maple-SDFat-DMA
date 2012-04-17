#ifndef Sd2Card_h
#define Sd2Card_h

#include <WProgram.h>
#include "SdInfo.h"
#include "HardwareSPI.h"
#include "spi.h"
#include "dma.h"

#define CS 4 // D10
#define CSPORT GPIOA

#define sbi(port,pin) gpio_write_bit(port,pin,1);
#define cbi(port,pin) gpio_write_bit(port,pin,0);
#define CS1 sbi(CSPORT,CS);
#define CS0 cbi(CSPORT,CS);

#define SD_PROTECT_BLOCK_ZERO 1 // Protect block zero from write if nonzero

#define SPI_BUFF_SIZE 512

uint16_t const SD_INIT_TIMEOUT = 2000; // init timeout ms
uint16_t const SD_ERASE_TIMEOUT = 10000; // erase timeout ms
uint16_t const SD_READ_TIMEOUT = 300; // read timeout ms
uint16_t const SD_WRITE_TIMEOUT = 600; // write time out ms

// SD card errors
uint8_t const SD_CARD_ERROR_CMD0 = 0X01; // CMD0 timeout
uint8_t const SD_CARD_ERROR_CMD8 = 0X02; // error to CMD8, not a valid SD card
uint8_t const SD_CARD_ERROR_CMD17 = 0X03; // error to CMD17 (read block)
uint8_t const SD_CARD_ERROR_CMD24 = 0X04; // error to CMD24 (write block)
uint8_t const SD_CARD_ERROR_CMD25 = 0X05; // WRITE_MULTIPLE_BLOCKS command failed
uint8_t const SD_CARD_ERROR_CMD58 = 0X06; // error to CMD58 (read OCR)
uint8_t const SD_CARD_ERROR_ACMD23 = 0X07; // SET_WR_BLK_ERASE_COUNT failed
uint8_t const SD_CARD_ERROR_ACMD41 = 0X08; // ACMD41 timeout
uint8_t const SD_CARD_ERROR_BAD_CSD = 0X09; //bad CSR version field
uint8_t const SD_CARD_ERROR_ERASE = 0X0A; // erase block group command failed
uint8_t const SD_CARD_ERROR_ERASE_SINGLE_BLOCK = 0X0B; // single block erase not available
uint8_t const SD_CARD_ERROR_ERASE_TIMEOUT = 0X0C; // Erase sequence timed out
uint8_t const SD_CARD_ERROR_READ = 0X0D; // error to read data
uint8_t const SD_CARD_ERROR_READ_REG = 0X0E; // read CID or CSD failed
uint8_t const SD_CARD_ERROR_READ_TIMEOUT = 0X0F; // read data timeout
uint8_t const SD_CARD_ERROR_STOP_TRAN = 0X10; // card did not accept STOP_TRAN_TOKEN
uint8_t const SD_CARD_ERROR_WRITE = 0X11; // error to write operation
uint8_t const SD_CARD_ERROR_WRITE_BLOCK_ZERO = 0X12; // attempt to write protected block zero
uint8_t const SD_CARD_ERROR_WRITE_MULTIPLE = 0X13; // multiple block write not available
uint8_t const SD_CARD_ERROR_WRITE_PROGRAMMING = 0X14; // error to CMD13
uint8_t const SD_CARD_ERROR_WRITE_TIMEOUT = 0X15; // write programming timeout
uint8_t const SD_CARD_ERROR_SCK_RATE = 0X16; // incorrect rate selected

uint8_t const SD_CARD_TYPE_SD1 = 1;
uint8_t const SD_CARD_TYPE_SD2 = 2;
uint8_t const SD_CARD_TYPE_SDHC = 3;

class Sd2Card : public HardwareSPI {
    public:
        Sd2Card();
        void spiSend(uint8_t b);
        uint8_t spiRec();
        uint32_t cardSize(void);
        uint8_t erase(uint32_t firstBlock, uint32_t lastBlock);
        uint8_t eraseSingleBlockEnable(void);
        uint8_t errorCode(void) const {return errorCode_;}
        uint8_t errorData(void) const {return status_;}
        uint8_t init();
        void partialBlockRead(uint8_t value);
        uint8_t partialBlockRead(void) const {return partialBlockRead_;}
        uint8_t readBlock(uint32_t block, uint8_t* dst);
        uint8_t readData(uint32_t block, uint16_t offset, uint16_t count, uint8_t* dst);
        uint8_t readCID(cid_t* cid) {
            return readRegister(CMD10, cid);
        }
        uint8_t readCSD(csd_t* csd) {
            return readRegister(CMD9, csd);
        }
        void readEnd(void);
        uint8_t type(void) const {return type_;}
        uint8_t writeBlock(uint32_t blockNumber, const uint8_t* src);
        uint8_t writeData(const uint8_t* src);
        uint8_t writeStart(uint32_t blockNumber, uint32_t eraseCount);
        uint8_t writeStop(void);
 
    private:
        uint32_t block_;
        uint8_t chipSelectPin_;
        uint8_t errorCode_;
        uint8_t inBlock_;
        uint16_t offset_;
        uint8_t partialBlockRead_;
        uint8_t status_;
        uint8_t type_;
        //pol
        uint8_t ack[SPI_BUFF_SIZE];
        // private functions
        uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
            cardCommand(CMD55, 0);
            return cardCommand(cmd, arg);
        }
        uint8_t cardCommand(uint8_t cmd, uint32_t arg);
        void error(uint8_t code) {errorCode_ = code;}
        uint8_t readRegister(uint8_t cmd, void* buf);
        uint8_t sendWriteCommand(uint32_t blockNumber, uint32_t eraseCount);
        void type(uint8_t value) {type_ = value;}
        uint8_t waitNotBusy(uint16_t timeoutMillis);
        uint8_t writeData(uint8_t token, const uint8_t* src);
        uint8_t waitStartBlock(void);
};
#endif
