#pragma once

void picovision_wxd_init();
void picovision_init();
void picovision_write_line(int y, uint32_t* data);
void picovision_flip();
void picovision_ack_dma();
