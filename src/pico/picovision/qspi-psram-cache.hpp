#include <unordered_map>
#include <queue>
#include <cstdint>
#include <math.h>

namespace ramshim {

  void __not_in_flash_func(read_page)(uint32_t a, uint32_t *buffer, uint32_t len_in_words) {
    ram.read_fast_blocking(a, buffer, len_in_words);
  }

#if 0
  void write_page(uint32_t a, uint32_t *buffer, uint32_t len_in_words) {
    ram.write(a, buffer, len_in_words << 2);
  }
#endif


  // ==========================================================================
  // page cache
  //
  // gives much better performance for contiguous or localised memory access
  // patterns. using a small amount of the available ram on the RP2040 as a
  // cache for reads and writes allows us to avoid some round trips to the
  // PSRAM chip.
  //
  // the cache implements a basic LRU strategy by maintaining a doubly linked
  // list of cache pages. on access (either read or write) if the page is
  // already cached then that page is moved to the head of the list - if the
  // page is not yet cached then a new page is fetched from the PSRAM ram chip
  // and the current tail page is evicted.
  // ==========================================================================

  // as per the datasheet (https://bit.ly/3e1Xk1q) we cannot chip select the
  // APS6404 PSRAM chip for more than 4us due to it needing to perform DRAM
  // refresh. this effectively limits the page size we can reasonably use to
  // 32 bytes. reading or writing 32 bytes takes ~3.2us with the current code
  // on an overclocked (to 250mhz) RP2040.
  constexpr static uint32_t _page_bits  = 5;
  constexpr static uint32_t _page_size  = 1 << _page_bits;

  // can be set to whatever makes sense for ram/performance tradeoff. by
  // default we allocate 256 pages which is 8KB.
  constexpr static uint32_t _page_count_bits = 8;
  constexpr static uint32_t _page_count = 1 << _page_count_bits;

  constexpr static uint32_t _address_mask  = _page_size - 1;
  constexpr static uint32_t _id_mask  = ~_address_mask;

  struct page_t {
    uint32_t id = ~0u;
    bool dirty = false;
    uint8_t __aligned(4) data[_page_size] = {0};

    // generic getter
    template <typename T>
    __always_inline T get(uint32_t a) {return *((T *)(data + a));}

    // generic setter
    template <typename T>
    __always_inline void set(uint32_t a, T v) {*((T *)(data + a)) = v; dirty = true;}

    // unsigned getters
    __always_inline uint32_t u32(uint32_t a) {return get<uint32_t>(a);}
    __always_inline uint16_t u16(uint32_t a) {return get<uint16_t>(a);}
    __always_inline uint8_t   u8(uint32_t a) {return get< uint8_t>(a);}

    // signed getters
    __always_inline int32_t  s32(uint32_t a) {return get<int32_t>(a);}
    __always_inline int16_t  s16(uint32_t a) {return get<int16_t>(a);}
    __always_inline int8_t    s8(uint32_t a) {return get< int8_t>(a);}

    // unsigned setters
    __always_inline void     u32(uint32_t a, uint32_t v) {set<uint32_t>(a, v);}
    __always_inline void     u16(uint32_t a, uint16_t v) {set<uint16_t>(a, v);}
    __always_inline void      u8(uint32_t a,  uint8_t v) {set< uint8_t>(a, v);}
  };

  struct cache_t {
    uint32_t _ram_start;

    critical_section_t _crit;

    // hidden storage for all of the possible pages
    page_t pages[_page_count];

    cache_t(uint32_t _ram_start) : _ram_start(_ram_start) {
      critical_section_init(&_crit);
    }

    __always_inline uint32_t address_to_id(uint32_t a) {
      // return the page id for the given address
      return ((a & _id_mask) - _ram_start);
    }

    __always_inline uint32_t address_to_offset(uint32_t a) {
      // return the page offset for the given address
      return a & _address_mask;
    }

    __always_inline uint32_t id_to_entry(uint32_t id) {
      return (id >> _page_bits) & ((1 << _page_count_bits) - 1);
    }

    __always_inline page_t *peek_page(uint32_t a) {
      uint32_t id = address_to_id(a);
      uint32_t entry = id_to_entry(id);
      return pages[entry].id == id ? &pages[entry] : nullptr;
    }

    page_t *__not_in_flash_func(fetch_page)(uint32_t a) {
      // convert requested address to start of page address
      uint32_t id = address_to_id(a);
      uint32_t entry = id_to_entry(id);
      page_t *page = &pages[entry];

      if (page->id != id) {
        // page is not cached so we need to load it from qspi ram

#if 0
        if (pages[entry].dirty) {
          write_page(page->id, (uint32_t*)page->data, _page_size >> 2);
        }
#endif

        // initialise page
        page->id = id;
        page->dirty = false;

        // load page
        read_page(page->id, (uint32_t*)page->data, _page_size >> 2);
      }

      return page;
    }

    // retrieve helpers
    __always_inline uint32_t u32(uint32_t a) {
      critical_section_enter_blocking(&_crit);
      uint32_t r = fetch_page(a)->u32(address_to_offset(a));
      critical_section_exit(&_crit);
      return r;
    }
    __always_inline uint16_t u16(uint32_t a) {
      critical_section_enter_blocking(&_crit);
      uint16_t r = fetch_page(a)->u16(address_to_offset(a));
      critical_section_exit(&_crit);
      return r;
    }
    __always_inline uint8_t   u8(uint32_t a) {
      critical_section_enter_blocking(&_crit);
      uint8_t r = fetch_page(a)->u8(address_to_offset(a));
      critical_section_exit(&_crit);
      return r;
    }
    __always_inline int32_t  s32(uint32_t a) {
      critical_section_enter_blocking(&_crit);
      int32_t r = fetch_page(a)->s32(address_to_offset(a));
      critical_section_exit(&_crit);
      return r;
    }
    __always_inline int16_t  s16(uint32_t a) {
      critical_section_enter_blocking(&_crit);
      int16_t r = fetch_page(a)->s16(address_to_offset(a));
      critical_section_exit(&_crit);
      return r;
    }
    __always_inline int8_t    s8(uint32_t a) {
      critical_section_enter_blocking(&_crit);
      int8_t r = fetch_page(a)->s8(address_to_offset(a));
      critical_section_exit(&_crit);
      return r;
    }

#if 0
    // store helpers
    __always_inline void u32(uint32_t a, uint32_t v) {fetch_page(a)->u32(address_to_offset(a), v);}
    __always_inline void u16(uint32_t a, uint16_t v) {fetch_page(a)->u16(address_to_offset(a), v);}
    __always_inline void  u8(uint32_t a, uint8_t  v) {fetch_page(a)->u8 (address_to_offset(a), v);}
#endif
  };

}