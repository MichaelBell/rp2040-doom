#pragma once

#include <stdint.h>

void picovision_wxd_init();
void picovision_init();
void picovision_write_palette(uint8_t* palette);
void picovision_write_line(int y, uint32_t* data);
void picovision_flip();
void picovision_notify_next_vsync();
void picovision_ack_dma();

uint8_t picovision_read_byte_from_cache(const uint8_t* addr);
uint16_t picovision_read_word_from_cache(const uint16_t* addr);
uint32_t picovision_read_dword_from_cache(const uint32_t* addr);
void picovision_read_bytes_from_cache(const uint8_t* addr, uint8_t* buf, uint32_t len);
void picovision_read_bytes(const uint8_t* addr, uint8_t* buf, uint32_t len);
void picovision_write(const uint8_t* addr, uint32_t* buf, uint32_t len);

void picovision_print_profile();