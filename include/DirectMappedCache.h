//
// Created by jwscoggins on 7/1/21.
//

#ifndef I960SXCHIPSET_DIRECTMAPPEDCACHE_H
#define I960SXCHIPSET_DIRECTMAPPEDCACHE_H
#include "MCUPlatform.h"

class DirectMappedCache {
public:
    struct CacheLine {
        bool valid_ = false;
        bool dirty_ = false;
        Address baseAddress = 0;
        SplitWord128 entry_ = 0;
    };
    static constexpr auto NumCacheLines = 256;
public:
    [[nodiscard]] static constexpr auto getTagIndex(Address address) noexcept { return (address & 0x00000FF0) >> 4; }
    static_assert (getTagIndex(0x10) == 1);
private:
    CacheLine cacheLines_[NumCacheLines] = { 0 };
};
#endif //I960SXCHIPSET_DIRECTMAPPEDCACHE_H
