#ifndef DARTT_INIT_H
#define DARTT_INIT_H

#include "cobs.h"
#include "dartt.h"
#include "dartt_sync.h"

#define SERIAL_BUFFER_SIZE 32
#define NUM_BYTES_COBS_OVERHEAD	2	//we have to tell dartt our serial buffers are smaller than they are, so the COBS layer has room to operate. This allows for functional multiple message handling with write_multi and read_multi for large configs


int tx_blocking(unsigned char addr, dartt_buffer_t * b, void * user_context, uint32_t timeout);
int rx_blocking(dartt_buffer_t * buf, void * user_context, uint32_t timeout);

#endif