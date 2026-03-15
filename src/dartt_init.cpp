#include "dartt_init.h"
#include <cstdio>

int tx_blocking(unsigned char addr, dartt_buffer_t * b, void * user_context, uint32_t timeout)
{
 	UdpState* udp_state = (UdpState*)(user_context);

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
	if (!udp_state->connected)
	{
		return -1;
	}	
	size_t bytes_sent = 0;
	TcsResult res = tcs_send(udp_state->socket, cb.buf, cb.length, TCS_FLAG_NONE, &bytes_sent);
	rc = (res == TCS_SUCCESS && bytes_sent == cb.length) ? (int)cb.length : -1;
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
	UdpState* udp_state = (UdpState*)(user_context);

	cobs_buf_t cb_enc =
	{
		.buf = udp_state->rx_cobs_mem,
		.size = sizeof(udp_state->rx_cobs_mem),
		.length = 0
	};

	int rc;
	if (!udp_state->connected)
	{
		return -1;
	}
	struct TcsAddress src;
	size_t bytes_received = 0;
	tcs_opt_receive_timeout_set(udp_state->socket, timeout);
	TcsResult res = tcs_receive_from(udp_state->socket, cb_enc.buf, cb_enc.size, TCS_FLAG_NONE, &src, &bytes_received);
	if (res == TCS_SUCCESS)
	{
		rc = (int)bytes_received;
	}
	else
	{
		rc = -2;
	}

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

bool udp_connect(UdpState* state)
{
	if (state->connected)
		udp_disconnect(state);

	state->socket = TCS_SOCKET_INVALID;
	TcsResult res = tcs_socket_preset(&state->socket, TCS_PRESET_UDP_IP4);
	if (res != TCS_SUCCESS)
	{
		printf("UDP: failed to create socket (%d)\n", res);
		return false;
	}

	// Resolve IP string to TcsAddress, then connect
	struct TcsAddress remote_addr = TCS_ADDRESS_NONE;
	size_t addr_count = 0;
	res = tcs_address_resolve(state->ip, TCS_AF_IP4, &remote_addr, 1, &addr_count);
	if (res != TCS_SUCCESS || addr_count == 0)
	{
		printf("UDP: failed to resolve address '%s' (%d)\n", state->ip, res);
		tcs_close(&state->socket);
		return false;
	}
	remote_addr.data.ip4.port = state->port;

	res = tcs_connect(state->socket, &remote_addr);
	if (res != TCS_SUCCESS)
	{
		printf("UDP: failed to connect to %s:%u (%d)\n", state->ip, state->port, res);
		tcs_close(&state->socket);
		return false;
	}

	state->connected = true;
	printf("UDP: connected. Targeting %s:%u\n", state->ip, state->port);
	return true;
}

void udp_disconnect(UdpState* state)
{
	if (state->socket != TCS_SOCKET_INVALID)
	{
		tcs_close(&state->socket);
	}
	state->connected = false;
	printf("UDP: disconnected\n");
}