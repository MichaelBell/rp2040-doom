#include <stdint.h>
#include <algorithm>
#include <stdio.h>

extern "C" {
#include "picovision.h"
}

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "swd_load.hpp"
#include "pico-stick.h"

#include "aps6404.hpp"

pimoroni::APS6404 ram;

#include "hard-fault-handler.hpp"

#include "ff.h"
static FATFS fs;
static FIL fil;

#define BUFFER_BYTES 320*168
extern uint8_t frame_buffer[2][BUFFER_BYTES];

#define BASE_ADDRESS 0x10000

static constexpr uint display_width = 360;
static constexpr uint display_height = 200;
static constexpr uint frame_width = 360;
static constexpr uint frame_height = 200;
static constexpr uint h_repeat = 2;
static constexpr uint v_repeat = 2;
#define NUM_PALETTES 0

// interface pins
static constexpr uint VSYNC  = 16;
static constexpr uint RAM_SEL = 8;
static constexpr uint I2C_SDA = 6;
static constexpr uint I2C_SCL = 7;

// I2C address and registers
static constexpr uint I2C_ADDR = 0x0D;
static constexpr uint I2C_REG_SET_RES = 0xFC;
static constexpr uint I2C_REG_START = 0xFD;
static constexpr uint I2C_REG_STOP = 0xFF;
static constexpr uint I2C_REG_GPIO = 0xC0;
static constexpr uint I2C_REG_LED = 0xC1;
static constexpr uint I2C_REG_GPIO29_MODE = 0xC2;
static constexpr uint I2C_REG_GPIO29_OUT = 0xC3;
static constexpr uint I2C_REG_GPIO29_ADC = 0xC4;
static constexpr uint I2C_REG_GPU_TEMP = 0xC6;
static constexpr uint I2C_REG_GPIO_HI = 0xC8;
static constexpr uint I2C_REG_GPIO_HI_OUT = 0xC9;
static constexpr uint I2C_REG_GPIO_HI_OE = 0xCA;
static constexpr uint I2C_REG_GPIO_HI_PULL_UP = 0xCB;
static constexpr uint I2C_REG_GPIO_HI_PULL_DOWN = 0xCC;
static constexpr uint I2C_REG_EDID = 0xFB;
static constexpr uint I2C_REG_PALETTE_INDEX = 0xF8;
static constexpr uint I2C_REG_SCROLL_BASE = 0xE0;

static uint8_t bank = 0;

  static volatile bool enable_switch_on_vsync = false;

// Used to re-trigger scanline fill
#define LOW_PRIO_IRQ 31

  static void vsync_callback() {
    if (gpio_get_irq_event_mask(VSYNC) & GPIO_IRQ_EDGE_RISE) {
      gpio_acknowledge_irq(VSYNC, GPIO_IRQ_EDGE_RISE);

      if (enable_switch_on_vsync) {
        // Toggle RAM_SEL pin
        critical_section_enter_blocking(&ram.mutex);
        ram.wait_for_finish_blocking();
        gpio_xor_mask(1 << RAM_SEL);
        critical_section_exit(&ram.mutex);

        enable_switch_on_vsync = false;
        *((io_rw_32 *) (PPB_BASE + M0PLUS_NVIC_ISPR_OFFSET)) = 1u << LOW_PRIO_IRQ;
      }
    }
  }

void write_header()
{
    constexpr int buf_size = 8;
    uint32_t buf[buf_size];
    uint32_t full_width = display_width * h_repeat;
    buf[0] = 0x4F434950;
    buf[1] = 0x01000101 + ((uint32_t)v_repeat << 16);
    buf[2] = full_width << 16;
    buf[3] = (uint32_t)display_height << 16;
    buf[4] = 0x00000001;
    buf[5] = 0x00000000 + display_height + ((uint32_t)bank << 24);
    buf[6] = 0x04000000 + NUM_PALETTES;
    ram.write(0, buf, 7 * 4);
    ram.wait_for_finish_blocking();

    constexpr int minx = 0;
    constexpr int miny = 0;
    constexpr int maxx = display_width;
    constexpr int maxy = display_height;

    uint addr = 4 * (7 + miny);
    uint line_type = (uint)1 << 27;
    for (int i = miny; i < maxy; i += buf_size) {
      int maxj = std::min(buf_size, maxy - i);
      for (int j = 0; j < maxj; ++j) {
        buf[j] = line_type + ((uint32_t)h_repeat << 24) + ((i + j) * 1024) + BASE_ADDRESS - 40;
      }
      ram.write(addr, buf, maxj * 4);
      ram.wait_for_finish_blocking();
      addr += 4 * maxj;
    }

    // Clear screen
    ram.write_repeat(BASE_ADDRESS - 40, 0, display_height * 1024);
}

void read_wxd()
{
    FRESULT fr = f_open(&fil, "/doom1.whx", FA_READ);
    if (fr != FR_OK) {
        printf("Failed to open WHX file, error: %d\n", fr);
        return;
    }

    uint32_t addr = 0x400000;
    uint8_t* buffer = frame_buffer[0];
    size_t bytes_read;
    do {
      fr = f_read(&fil, buffer, 1024, &bytes_read);
      if (fr != FR_OK) {
          printf("Failed to read data, error: %d\n", fr);
      }
      if (bytes_read > 0) {
        ram.write(addr, (uint32_t*)buffer, bytes_read);
        addr += bytes_read;
      }
    } while (bytes_read == 1024);

    f_close(&fil);
}

void picovision_wxd_init()
{
    swd_load_program(section_addresses, section_data, section_data_len, sizeof(section_addresses) / sizeof(section_addresses[0]), 0x20000001, 0x15004000, true);

    FRESULT fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
      printf("Failed to mount SD card, error: %d\n", fr);
      return;
    }

    gpio_init(RAM_SEL);
    gpio_put(RAM_SEL, 0);
    gpio_set_dir(RAM_SEL, GPIO_OUT);

    gpio_init(VSYNC);
    gpio_set_dir(VSYNC, GPIO_IN);

    gpio_put(RAM_SEL, 0);
    ram.init();
    bank = 0;
    write_header();
    read_wxd();
    sleep_us(100);

    gpio_put(RAM_SEL, 1);
    ram.init();
    bank = 1;
    write_header();
    read_wxd();
    sleep_us(100);

    bank = 0;
    gpio_put(RAM_SEL, 0);
    sleep_us(100);
}

void picovision_wxd_read()
{

}

void picovision_init()
{
    gpio_set_irq_enabled(VSYNC, GPIO_IRQ_EDGE_RISE, true);
    irq_set_exclusive_handler(IO_IRQ_BANK0, vsync_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); gpio_pull_up(I2C_SDA);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); gpio_pull_up(I2C_SCL);

    //mp_printf(&mp_plat_print, "Start I2C\n");

    {
        uint8_t buffer[2] = {I2C_REG_SET_RES, 2};
        i2c_write_blocking(i2c1, I2C_ADDR, buffer, 2, false);
    }

    {
        uint8_t buffer[2] = {I2C_REG_START, 1};
        i2c_write_blocking(i2c1, I2C_ADDR, buffer, 2, false);
    }
}

void picovision_write_line(int y, uint32_t* data)
{
    // TODO
    #if 0
    while (enable_switch_on_vsync) {
      // Waiting for IRQ handler to do flip.
      ;
    }
    #endif

    uint32_t addr = BASE_ADDRESS + y * 1024;
    ram.write_fast_irq(addr, data, 320*2);
}

void picovision_flip()
{
    bank ^= 1;
    ram.wait_for_finish_blocking();

    enable_switch_on_vsync = true;

    // TODO
    //while (gpio_get(VSYNC) == 0);
    //gpio_xor_mask(1 << RAM_SEL);    
}

void picovision_ack_dma()
{
    ram.ack_dma_irq();
}