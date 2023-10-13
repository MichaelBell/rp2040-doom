#include <algorithm>
#include "aps6404.hpp"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

#ifndef NO_QSTR
#include "aps6404.pio.h"
#endif

#ifndef MICROPY_BUILD_TYPE
#define mp_printf(_, ...) printf(__VA_ARGS__);
#else
extern "C" {
#include "py/runtime.h"
}
#endif

namespace {
    void aps6404_reset_program_init(PIO pio, uint sm, uint offset, uint csn, uint mosi) {
        uint miso = mosi + 1;
        pio_gpio_init(pio, csn);
        pio_gpio_init(pio, csn + 1);
        pio_gpio_init(pio, mosi);
        pio_sm_set_consecutive_pindirs(pio, sm, csn, 2, true);
        pio_sm_set_consecutive_pindirs(pio, sm, mosi, 1, true);
        pio_sm_set_consecutive_pindirs(pio, sm, miso, 1, false);

        pio_sm_config c = sram_reset_program_get_default_config(offset);
        sm_config_set_in_pins(&c, miso);
        sm_config_set_in_shift(&c, false, true, 32);
        sm_config_set_out_pins(&c, mosi, 1);
        sm_config_set_out_shift(&c, false, true, 32);
        sm_config_set_sideset_pins(&c, csn);
        sm_config_set_clkdiv(&c, 4.f);

        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }
    void aps6404_program_init(PIO pio, uint sm, uint offset, uint csn, uint mosi, bool slow, bool fast, bool reset) {
        pio_gpio_init(pio, csn);
        pio_gpio_init(pio, csn + 1);
        pio_gpio_init(pio, mosi);
        pio_gpio_init(pio, mosi + 1);
        pio_gpio_init(pio, mosi + 2);
        pio_gpio_init(pio, mosi + 3);
        pio_sm_set_consecutive_pindirs(pio, sm, csn, 2, true);
        pio_sm_set_consecutive_pindirs(pio, sm, mosi, 4, false);

        pio_sm_config c = slow ? sram_slow_program_get_default_config(offset) : 
                        fast ? sram_fast_program_get_default_config(offset) : 
                        reset ? sram_reset_qpi_program_get_default_config(offset) :
                        sram_program_get_default_config(offset);
        sm_config_set_in_pins(&c, mosi);
        sm_config_set_in_shift(&c, false, true, 32);
        sm_config_set_out_pins(&c, mosi, 4);
        sm_config_set_out_shift(&c, false, true, 32);
        sm_config_set_set_pins(&c, mosi, 4);
        sm_config_set_sideset_pins(&c, csn);

        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }
}

static const pio_program* pio_prog[2] = {nullptr, nullptr};
static uint16_t pio_offset[2] = {0xffff, 0xffff};

const void pio_remove_exclusive_program(PIO pio) {
    uint8_t pio_index = pio == pio0 ? 0 : 1;
    const pio_program* current_program = pio_prog[pio_index];
    uint16_t current_offset = pio_offset[pio_index];
    if(current_program) {
        pio_remove_program(pio, current_program, current_offset);
        pio_prog[pio_index] = nullptr;
        pio_offset[pio_index] = 0xffff;
    }
}

const uint16_t pio_change_exclusive_program(PIO pio, const pio_program* prog) {
    pio_remove_exclusive_program(pio);
    uint8_t pio_index = pio == pio0 ? 0 : 1;
    pio_prog[pio_index] = prog;
    pio_offset[pio_index] = pio_add_program(pio, prog);
    return pio_offset[pio_index];
};

namespace pimoroni {
    APS6404::APS6404(uint pin_csn, uint pin_d0, PIO pio)
                : pin_csn(pin_csn)
                , pin_d0(pin_d0)
                , pio(pio)
    {
        // Initialize data pins
        for (int i = 0; i < 4; ++i) {
            gpio_init(pin_d0 + i);
            gpio_disable_pulls(pin_d0 + i);
        }

        pio_sm = pio_claim_unused_sm(pio, true);

        // Claim DMA channels
        dma_channel = dma_claim_unused_channel(true);
        read_cmd_dma_channel = dma_claim_unused_channel(true);
        setup_dma_config();

        // User code must setup a handler for the DMA, but by default the channel is set
        // to quiet moe so does not trigger.
        dma_channel_set_irq0_enabled(dma_channel, true);

        critical_section_init(&mutex);
    }

    void APS6404::init() {
        pio_sm_set_enabled(pio, pio_sm, false);

        pio_offset = pio_change_exclusive_program(pio, &sram_reset_program);
        aps6404_reset_program_init(pio, pio_sm, pio_offset, pin_csn, pin_d0);

        sleep_us(200);
        pio_sm_put_blocking(pio, pio_sm, 0x00000007u);
        pio_sm_put_blocking(pio, pio_sm, 0x66000000u);
        pio_sm_put_blocking(pio, pio_sm, 0x00000007u);
        pio_sm_put_blocking(pio, pio_sm, 0x99000000u);
        pio_sm_put_blocking(pio, pio_sm, 0x00000007u);
        pio_sm_put_blocking(pio, pio_sm, 0x35000000u);
        sleep_us(500);

        adjust_clock();
    }

    void APS6404::set_qpi() {
        pio_sm_set_enabled(pio, pio_sm, false);

        pio_offset = pio_change_exclusive_program(pio, &sram_reset_program);
        aps6404_reset_program_init(pio, pio_sm, pio_offset, pin_csn, pin_d0);
        pio_sm_put_blocking(pio, pio_sm, 0x00000007u);
        pio_sm_put_blocking(pio, pio_sm, 0x35000000u);

        while (!pio_sm_is_tx_fifo_empty(pio, pio_sm) || pio->sm[pio_sm].addr != pio_offset);

        adjust_clock();
    }

    void APS6404::set_spi() {
        pio_sm_set_enabled(pio, pio_sm, false);

        pio_offset = pio_change_exclusive_program(pio, &sram_reset_qpi_program);
        aps6404_program_init(pio, pio_sm, pio_offset, pin_csn, pin_d0, false, false, true);
        pio_sm_put_blocking(pio, pio_sm, 0x00000001u);
        pio_sm_put_blocking(pio, pio_sm, 0xF5000000u);
    }

    void APS6404::adjust_clock() {
        pio_sm_set_enabled(pio, pio_sm, false);
        uint32_t clock_hz = clock_get_hz(clk_sys);

        if (clock_hz > 296000000) {
            pio_offset = pio_change_exclusive_program(pio, &sram_fast_program);
            aps6404_program_init(pio, pio_sm, pio_offset, pin_csn, pin_d0, false, true, false);
        }
        else if (clock_hz < 130000000) {
            pio_offset = pio_change_exclusive_program(pio, &sram_slow_program);
            aps6404_program_init(pio, pio_sm, pio_offset, pin_csn, pin_d0, true, false, false);
        }
        else {
            pio_offset = pio_change_exclusive_program(pio, &sram_program);
            aps6404_program_init(pio, pio_sm, pio_offset, pin_csn, pin_d0, false, false, false);
        }

        last_cmd_was_write = false;
        page_smashing_ok = clock_hz <= 168000000;
    }

    

    void APS6404::setup_dma_config() {
        dma_channel_config c = dma_channel_get_default_config(read_cmd_dma_channel);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio, pio_sm, true));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        
        dma_channel_configure(
            read_cmd_dma_channel, &c,
            &pio->txf[pio_sm],
            multi_read_cmd_buffer,
            0,
            false
        );

        write_config = dma_channel_get_default_config(dma_channel);
        channel_config_set_read_increment(&write_config, true);
        channel_config_set_write_increment(&write_config, false);
        channel_config_set_dreq(&write_config, pio_get_dreq(pio, pio_sm, true));
        channel_config_set_transfer_data_size(&write_config, DMA_SIZE_32);
        channel_config_set_bswap(&write_config, true);
        channel_config_set_irq_quiet(&write_config, true);

        read_config = dma_channel_get_default_config(dma_channel);
        channel_config_set_read_increment(&read_config, false);
        channel_config_set_write_increment(&read_config, true);
        channel_config_set_dreq(&read_config, pio_get_dreq(pio, pio_sm, false));
        channel_config_set_transfer_data_size(&read_config, DMA_SIZE_32);
        channel_config_set_bswap(&read_config, true);
        channel_config_set_irq_quiet(&read_config, true);
    }

    void APS6404::write_no_page_crossing(uint32_t addr, uint32_t* data, uint32_t len_in_bytes) {
        int len = len_in_bytes;
        int page_len = PAGE_SIZE - (addr & (PAGE_SIZE - 1));

        if ((page_len & 3) != 0) {
            while (len > page_len) {
                wait_for_finish_blocking();
                hw_set_bits(&dma_hw->ch[dma_channel].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);

                pio_sm_put_blocking(pio, pio_sm, (page_len << 1) - 1);
                pio_sm_put_blocking(pio, pio_sm, 0x38000000u | addr);
                pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_write);

                dma_channel_transfer_from_buffer_now(dma_channel, data, (page_len >> 2) + 1);

                len -= page_len;
                addr += page_len;
                data += page_len >> 2;
                int bytes_sent_last_word = page_len & 3;
                page_len = std::min(4 - bytes_sent_last_word, len);
                
                dma_channel_wait_for_finish_blocking(dma_channel);
                pio_sm_put_blocking(pio, pio_sm, (page_len << 1) - 1);
                pio_sm_put_blocking(pio, pio_sm, 0x38000000u | addr);
                pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_write);
                pio_sm_put_blocking(pio, pio_sm, __builtin_bswap32(*data >> (8 * bytes_sent_last_word)));
                
                addr += page_len;
                len -= page_len;
                ++data;
                page_len = PAGE_SIZE - page_len;
            }
        }

        for (page_len = std::min(page_len, len);
            len > 0; 
            addr += page_len, data += page_len >> 2, len -= page_len, page_len = std::min(PAGE_SIZE, len))
        {
            wait_for_finish_blocking();
            hw_set_bits(&dma_hw->ch[dma_channel].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);

            pio_sm_put_blocking(pio, pio_sm, (page_len << 1) - 1);
            pio_sm_put_blocking(pio, pio_sm, 0x38000000u | addr);
            pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_write);

            dma_channel_transfer_from_buffer_now(dma_channel, data, (page_len >> 2) + 1);
        }
    }

    void APS6404::write(uint32_t addr, uint32_t* data, uint32_t len_in_bytes) {
        if (!last_cmd_was_write) {
            last_cmd_was_write = true;
            wait_for_finish_blocking();
            dma_channel_set_write_addr(dma_channel, &pio->txf[pio_sm], false);
            dma_hw->ch[dma_channel].al1_ctrl = write_config.ctrl;
        }

        if (!page_smashing_ok) {
            write_no_page_crossing(addr, data, len_in_bytes);
            return;
        }

        for (int len = len_in_bytes, page_len = std::min(PAGE_SIZE, len); 
            len > 0; 
            addr += page_len, data += page_len >> 2, len -= page_len, page_len = std::min(PAGE_SIZE, len))
        {
            wait_for_finish_blocking();
            hw_set_bits(&dma_hw->ch[dma_channel].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS);

            pio_sm_put_blocking(pio, pio_sm, (page_len << 1) - 1);
            pio_sm_put_blocking(pio, pio_sm, 0x38000000u | addr);
            pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_write);

            dma_channel_transfer_from_buffer_now(dma_channel, data, (page_len >> 2) + 1);
        }
    }

    void APS6404::write_fast_irq(uint32_t addr, uint32_t* data, uint32_t len_in_bytes) {
        critical_section_enter_blocking(&mutex);
        if (!last_cmd_was_write) {
            last_cmd_was_write = true;
            dma_channel_wait_for_finish_blocking(dma_channel);
            dma_channel_set_write_addr(dma_channel, &pio->txf[pio_sm], false);
        }
        else {
            dma_channel_wait_for_finish_blocking(dma_channel);
        }
        
        dma_hw->ch[dma_channel].al1_ctrl = write_config.ctrl & (~DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS);
        
        pio_sm_put_blocking(pio, pio_sm, (len_in_bytes << 1) - 1);
        pio_sm_put_blocking(pio, pio_sm, 0x38000000u | addr);
        pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_write);

        dma_channel_transfer_from_buffer_now(dma_channel, data, (len_in_bytes >> 2) + 1);
        critical_section_exit(&mutex);
    }

    void APS6404::write_repeat(uint32_t addr, uint32_t data, uint32_t len_in_bytes) {
        if (!last_cmd_was_write) {
            last_cmd_was_write = true;
            dma_channel_set_write_addr(dma_channel, &pio->txf[pio_sm], false);
            dma_hw->ch[dma_channel].al1_ctrl = write_config.ctrl;
        }

        int first_page_len = PAGE_SIZE;
        if (!page_smashing_ok) {
            first_page_len -= (addr & (PAGE_SIZE - 1));
            if ((first_page_len & 3) != 0 && (int)len_in_bytes > first_page_len) {
                wait_for_finish_blocking();
                hw_clear_bits(&dma_hw->ch[dma_channel].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);

                pio_sm_put_blocking(pio, pio_sm, (first_page_len << 1) - 1);
                pio_sm_put_blocking(pio, pio_sm, 0x38000000u | addr);
                pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_write);

                dma_channel_transfer_from_buffer_now(dma_channel, &data, (first_page_len >> 2) + 1);

                len_in_bytes -= first_page_len;
                addr += first_page_len;
                int bytes_sent_last_word = first_page_len & 3;
                first_page_len = PAGE_SIZE;
                data = (data >> (8 * bytes_sent_last_word)) | (data << (32 - (8 * bytes_sent_last_word)));
            }
        }

        for (int len = len_in_bytes, page_len = std::min(first_page_len, len); 
             len > 0; 
             addr += page_len, len -= page_len, page_len = std::min(PAGE_SIZE, len))
        {
            wait_for_finish_blocking();
            hw_clear_bits(&dma_hw->ch[dma_channel].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);
            repeat_data = data;

            pio_sm_put_blocking(pio, pio_sm, (page_len << 1) - 1);
            pio_sm_put_blocking(pio, pio_sm, 0x38000000u | addr);
            pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_write);

            dma_channel_transfer_from_buffer_now(dma_channel, &repeat_data, (page_len >> 2) + 1);
        }
    }

    void APS6404::read(uint32_t addr, uint32_t* read_buf, uint32_t len_in_words) {
        start_read(read_buf, len_in_words);

        uint32_t first_page_len = PAGE_SIZE;
        if (!page_smashing_ok) {
            first_page_len -= (addr & (PAGE_SIZE - 1));
        }

        if (first_page_len >= len_in_words << 2) {
            pio_sm_put_blocking(pio, pio_sm, (len_in_words * 8) - 4);
            pio_sm_put_blocking(pio, pio_sm, 0xeb000000u | addr);
            pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_read);
        }
        else {
            uint32_t* cmd_buf = add_read_to_cmd_buffer(multi_read_cmd_buffer, addr, len_in_words);
            dma_channel_transfer_from_buffer_now(read_cmd_dma_channel, multi_read_cmd_buffer, cmd_buf - multi_read_cmd_buffer);
        }
    }

    void APS6404::read_fast_blocking(uint32_t addr, uint32_t* read_buf, uint32_t len_in_words) {
        critical_section_enter_blocking(&mutex);
        last_cmd_was_write = false;
        wait_for_finish_blocking();

        dma_channel_configure(
            dma_channel, &read_config,
            read_buf,
            &pio->rxf[pio_sm],
            len_in_words,
            true
        );

        pio_sm_put_blocking(pio, pio_sm, (len_in_words * 8) - 4);
        pio_sm_put_blocking(pio, pio_sm, 0xeb000000u | addr);
        pio_sm_put_blocking(pio, pio_sm, pio_offset + sram_offset_do_read);
        wait_for_finish_blocking();
        critical_section_exit(&mutex);
    }

    void APS6404::multi_read(uint32_t* addresses, uint32_t* lengths, uint32_t num_reads, uint32_t* read_buf, int chain_channel) {
        uint32_t total_len = 0;
        uint32_t* cmd_buf = multi_read_cmd_buffer;
        for (uint32_t i = 0; i < num_reads; ++i) {
            total_len += lengths[i];
            cmd_buf = add_read_to_cmd_buffer(cmd_buf, addresses[i], lengths[i]);
        }

        start_read(read_buf, total_len, chain_channel);

        dma_channel_transfer_from_buffer_now(read_cmd_dma_channel, multi_read_cmd_buffer, cmd_buf - multi_read_cmd_buffer);
    }

    void APS6404::start_read(uint32_t* read_buf, uint32_t total_len_in_words, int chain_channel) {
        dma_channel_config c = read_config;
        if (chain_channel >= 0) {
            channel_config_set_chain_to(&c, chain_channel);
        }
        
        last_cmd_was_write = false;
        wait_for_finish_blocking();

        dma_channel_configure(
            dma_channel, &c,
            read_buf,
            &pio->rxf[pio_sm],
            total_len_in_words,
            true
        );
    }

    uint32_t* APS6404::add_read_to_cmd_buffer(uint32_t* cmd_buf, uint32_t addr, uint32_t len_in_words) {
        int32_t len_remaining = len_in_words << 2;
        uint32_t align = page_smashing_ok ? 0 : (addr & (PAGE_SIZE - 1));
        uint32_t len = std::min((PAGE_SIZE - align), (uint32_t)len_remaining);

        while (true) {
            if (len < 2) {
                *cmd_buf++ = 0;
                *cmd_buf++ = 0xeb000000u | addr;
                *cmd_buf++ = pio_offset + sram_offset_do_read_one;
            }
            else {
                *cmd_buf++ = (len * 2) - 4;
                *cmd_buf++ = 0xeb000000u | addr;
                *cmd_buf++ = pio_offset + sram_offset_do_read;
            }
            len_remaining -= len;
            addr += len;

            if (len_remaining <= 0) break;

            len = len_remaining;
            if (len > PAGE_SIZE) len = PAGE_SIZE;
        }

        return cmd_buf;
    }    
}
