#ifndef _SdVolume_h
#define _SdVolume_h

class SdVolume {
    public:
        /** Create an instance of SdVolume */
        SdVolume(void) :allocSearchStart_(2), fatType_(0) {}
        /** Clear the cache and returns a pointer to the cache.  Used by the WaveRP
         *  recorder to do raw write to the SD card.  Not for normal apps.
         */
        static uint8_t* cacheClear(void) {
            cacheFlush();
            cacheBlockNumber_ = 0XFFFFFFFF;
            return cacheBuffer_.data;
        }
        /**
         * Initialize a FAT volume.  Try partition one first then try super
         * floppy format.
         *
         * \param[in] dev The Sd2Card where the volume is located.
         *
         * \return The value one, true, is returned for success and
         * the value zero, false, is returned for failure.  Reasons for
         * failure include not finding a valid partition, not finding a valid
         * FAT file system or an I/O error.
         */
        uint8_t init(Sd2Card* dev) { return init(dev, 1) ? true : init(dev, 0);}
        uint8_t init(Sd2Card* dev, uint8_t part);
        
        // inline functions that return volume info
        /** \return The volume's cluster size in blocks. */
        uint8_t blocksPerCluster(void) const {return blocksPerCluster_;}
        /** \return The number of blocks in one FAT. */
        uint32_t blocksPerFat(void)  const {return blocksPerFat_;}
        /** \return The total number of clusters in the volume. */
        uint32_t clusterCount(void) const {return clusterCount_;}
        /** \return The shift count required to multiply by blocksPerCluster. */
        uint8_t clusterSizeShift(void) const {return clusterSizeShift_;}
        /** \return The logical block number for the start of file data. */
        uint32_t dataStartBlock(void) const {return dataStartBlock_;}
        /** \return The number of FAT structures on the volume. */
        uint8_t fatCount(void) const {return fatCount_;}
        /** \return The logical block number for the start of the first FAT. */
        uint32_t fatStartBlock(void) const {return fatStartBlock_;}
        /** \return The FAT type of the volume. Values are 12, 16 or 32. */
        uint8_t fatType(void) const {return fatType_;}
        /** \return The number of entries in the root directory for FAT16 volumes. */
        uint32_t rootDirEntryCount(void) const {return rootDirEntryCount_;}
        /** \return The logical block number for the start of the root directory
         on FAT16 volumes or the first cluster number on FAT32 volumes. */
        uint32_t rootDirStart(void) const {return rootDirStart_;}
        /** return a pointer to the Sd2Card object for this volume */
        static Sd2Card* sdCard(void) {return sdCard_;}
    private:
        // Allow SdFile access to SdVolume private data.
        friend class SdFile;
        // value for action argument in cacheRawBlock to indicate read from cache
        static uint8_t const CACHE_FOR_READ = 0;
        // value for action argument in cacheRawBlock to indicate cache dirty
        static uint8_t const CACHE_FOR_WRITE = 1;
        static cache_t cacheBuffer_;        // 512 byte cache for device blocks
        static uint32_t cacheBlockNumber_;  // Logical number of block in the cache
        static Sd2Card* sdCard_;            // Sd2Card object for cache
        static uint8_t cacheDirty_;         // cacheFlush() will write block if true
        static uint32_t cacheMirrorBlock_;  // block number for mirror FAT
        
        uint32_t allocSearchStart_;   // start cluster for alloc search
        uint8_t blocksPerCluster_;    // cluster size in blocks
        uint32_t blocksPerFat_;       // FAT size in blocks
        uint32_t clusterCount_;       // clusters in one FAT
        uint8_t clusterSizeShift_;    // shift to convert cluster count to block count
        uint32_t dataStartBlock_;     // first data block number
        uint8_t fatCount_;            // number of FATs on volume
        uint32_t fatStartBlock_;      // start block for first FAT
        uint8_t fatType_;             // volume type (12, 16, OR 32)
        uint16_t rootDirEntryCount_;  // number of entries in FAT16 root dir
        uint32_t rootDirStart_;       // root start block for FAT16, cluster for FAT32
        //----------------------------------------------------------------------------
        uint8_t allocContiguous(uint32_t count, uint32_t* curCluster);
        uint8_t blockOfCluster(uint32_t position) const {
            return (position >> 9) & (blocksPerCluster_ - 1);
        }
        uint32_t clusterStartBlock(uint32_t cluster) const {
            return dataStartBlock_ + ((cluster - 2) << clusterSizeShift_);
        }
        uint32_t blockNumber(uint32_t cluster, uint32_t position) const {
            return clusterStartBlock(cluster) + blockOfCluster(position);
        }
        static uint8_t cacheFlush(void);
        static uint8_t cacheRawBlock(uint32_t blockNumber, uint8_t action);
        static void cacheSetDirty(void) {cacheDirty_ |= CACHE_FOR_WRITE;}
        static uint8_t cacheZeroBlock(uint32_t blockNumber);
        uint8_t chainSize(uint32_t beginCluster, uint32_t* size) const;
        uint8_t fatGet(uint32_t cluster, uint32_t* value) const;
        uint8_t fatPut(uint32_t cluster, uint32_t value);
        uint8_t fatPutEOC(uint32_t cluster) {
            return fatPut(cluster, 0x0FFFFFFF);
        }
        uint8_t freeChain(uint32_t cluster);
        uint8_t isEOC(uint32_t cluster) const {
            return  cluster >= (fatType_ == 16 ? FAT16EOC_MIN : FAT32EOC_MIN);
        }
        uint8_t readBlock(uint32_t block, uint8_t* dst) {
            return sdCard_->readBlock(block, dst);
        }
        uint8_t readData(uint32_t block, uint16_t offset,uint16_t count, uint8_t* dst) {
            return sdCard_->readData(block, offset, count, dst);
        }
        uint8_t writeBlock(uint32_t block, const uint8_t* dst) {
            return sdCard_->writeBlock(block, dst);
        }
};

#endif
