#include "serial_callbacks.h"
#include "serial.h"
#include <cstdio>

int tx_blocking(unsigned char addr, dartt_buffer_t * b, void * user_context, uint32_t timeout)
{
	if(user_context == NULL)
	{
		return -2;
	}
	Serial * pser = (Serial*)(user_context);
	if(pser->connected() == false)
	{
		return -2;
	}
	cobs_buf_t cb = {
		.buf = b->buf,
		.size = SERIAL_BUFFER_SIZE,	//important! this has to be the true size - b doesn't know what size it actually has. Use SERIAL_BUFFER_SIZE as source of truth for size since sizeof(var) is out of scope
		.length = b->len,
		.encoded_state = COBS_DECODED
	};
	int rc = cobs_encode_single_buffer(&cb);
	if (rc != 0)
	{
		return rc;
	}

	rc = pser->write(cb.buf, (int)cb.length);
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
	Serial * pser = (Serial*)(user_context);
	if(pser->connected() == false)
	{
		return -2;
	}

	cobs_buf_t cb_enc =
	{
		.buf = buf->buf,
		.size = SERIAL_BUFFER_SIZE,	//important! this has to be the true size - b doesn't know what size it actually has. Use SERIAL_BUFFER_SIZE as source of truth for size since sizeof(var) is out of scope
		.length = 0
	};

	int rc;
	rc = pser->read_until_delimiter(cb_enc.buf, cb_enc.size, 0, timeout);

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
