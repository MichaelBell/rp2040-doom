#include <cstdint>
#include <math.h>

namespace ramshim {

  void __not_in_flash_func(read_page)(uint32_t a, uint32_t *buffer, uint32_t len_in_words) {
    ram.read_fast_blocking(a, buffer, len_in_words);
  }


  // simple page cache

  constexpr static uint32_t _page_bits  = 5;
  constexpr static uint32_t _page_size  = 1 << _page_bits;

  // can be set to whatever makes sense for ram/performance tradeoff. by
  // default we allocate 256 pages which is 8KB.
  constexpr static uint32_t _page_count_bits = 10;
  constexpr static uint32_t _page_count = 1 << _page_count_bits;

  constexpr static uint32_t _address_mask  = _page_size - 1;
  constexpr static uint32_t _id_mask  = (~_address_mask) & (_ram_size - 1);

  struct page_t {
    uint32_t id = ~0u;
    uint8_t __aligned(4) data[_page_size] = {0};

    // generic getter
    template <typename T>
    __always_inline T get(uint32_t a) {return *((T *)(data + a));}

    // unsigned getters
    __always_inline uint32_t u32(uint32_t a) {return get<uint32_t>(a);}
    __always_inline uint16_t u16(uint32_t a) {return get<uint16_t>(a);}
    __always_inline uint8_t   u8(uint32_t a) {return get< uint8_t>(a);}

    // signed getters
    __always_inline int32_t  s32(uint32_t a) {return get<int32_t>(a);}
    __always_inline int16_t  s16(uint32_t a) {return get<int16_t>(a);}
    __always_inline int8_t    s8(uint32_t a) {return get< int8_t>(a);}
  };

  struct cache_t {
    critical_section_t _crit;

    // hidden storage for all of the possible pages
    page_t pages[_page_count];

    cache_t() {
      critical_section_init(&_crit);
    }

    __always_inline uint32_t address_to_id(uint32_t a) {
      // return the page id for the given address
      return a & _id_mask;
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

        // initialise page
        page->id = id;

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

    __always_inline void read_bytes(uint32_t a, uint8_t* buf, uint32_t len) {
      critical_section_enter_blocking(&_crit);

      auto* page = fetch_page(a);
      uint8_t* data = &page->data[address_to_offset(a)];

      while (len--) {
        if (address_to_id(a) != page->id) {
          page = fetch_page(a);
          data = page->data;
        }
        *buf++ = *data++;
        ++a;
      }

      critical_section_exit(&_crit);
    }

    __always_inline void read_bytes_uncached(uint32_t a, uint8_t* buf, uint32_t len) {
      uint8_t __aligned(4) read_buf[_page_size];

      uint32_t id = address_to_id(a);
      read_page(id, (uint32_t*)read_buf, _page_size >> 2);

      uint8_t* data = &read_buf[address_to_offset(a)];

      while (len--) {
        if (address_to_id(a) != id) {
          id = address_to_id(a);
          read_page(id, (uint32_t*)read_buf, _page_size >> 2);
          data = read_buf;
        }
        *buf++ = *data++;
        ++a;
      }
   }

  };

}