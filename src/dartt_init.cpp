#include "dartt_init.h"
#include "serial.h"
#include <cstdio>

unsigned char tx_mem[SERIAL_BUFFER_SIZE] = {};
unsigned char rx_dartt_mem[SERIAL_BUFFER_SIZE] = {};
unsigned char rx_cobs_mem[SERIAL_BUFFER_SIZE] = {};

int tx_blocking(unsigned char addr, dartt_buffer_t * b, void * user_context, uint32_t timeout)
{
	if(user_context == NULL)
	{
		return -2;
	}
 	Serial * serial = (Serial*)user_context;

	cobs_buf_t cb = {
		.buf = b->buf,
		.size = b->size,
		.length = b->len,
		.encoded_state = COBS_DECODED
	};
	int rc = cobs_encode_single_buffer(&cb);
	if (rc != 0)
	{
		return rc;
	}
	rc = serial->write(cb.buf, (int)cb.length);
	if(rc == (int)cb.length)
	{
		return DARTT_PROTOCOL_SUCCESS;
	}
	else
	{
		return -1;
	}
}

int rx_blocking(dartt_buffer_t * buf, void * user_context, uint32_t timeout)
{
	if(user_context == NULL)
	{
		return -2;
	}

 	Serial * serial = (Serial*)user_context;

	cobs_buf_t cb_enc =
	{
		.buf = rx_cobs_mem,
		.size = sizeof(rx_cobs_mem),
		.length = 0
	};

	int rc;
	if (! (serial->connected()))
	{
		return -1;
	}
	rc = serial->read_until_delimiter(cb_enc.buf, cb_enc.size, 0, timeout);

	if (rc >= 0)
	{
		cb_enc.length = rc;	//load encoded length (raw buffer)
	}
	else if (rc == -2)
	{
		return -7;
	}
	else
	{
		return -1;
	}

	cobs_buf_t cb_dec =
	{
		.buf = buf->buf,
		.size = buf->size,
		.length = 0
	};
	rc = cobs_decode_double_buffer(&cb_enc, &cb_dec);
	buf->len = cb_dec.length;	//critical - we are aliasing this read buffer in sync, but must update the length to the cobs decoded value

	if (rc != COBS_SUCCESS)
	{
		return rc;
	}
	else
	{
		return DARTT_PROTOCOL_SUCCESS;
	}
    
}
