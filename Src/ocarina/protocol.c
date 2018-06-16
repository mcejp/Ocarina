
#include "ocarina/ocarina.h"
#include "ocarina/protocol.h"

#include <eforce/ringbuf.h>

#include <string.h>

static const char* s_interface_id;
static const uint8_t sync[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
static const uint8_t heartbeat[] = {0xAF};

static uint8_t in_buf[128];
static ringbuf_t in_rb;

static uint8_t out_buf[128];
static ringbuf_t out_rb;

static uint8_t usb_buf[64];

void protocolInit(const char* interface_id) {
	s_interface_id = interface_id;

	in_rb.data = in_buf;
	in_rb.size = sizeof(in_buf);
	in_rb.readpos = 0;
	in_rb.writepos = 0;

	out_rb.data = out_buf;
	out_rb.size = sizeof(out_buf);
	out_rb.readpos = 0;
	out_rb.writepos = 0;
}

void protocolMessageReceivedStd(uint16_t sid, const uint8_t* data, size_t length) {
	uint8_t frame[4 + 8];
	size_t frame_size = 4 + length;
	size_t i;

	frame[0] = DTH_MESSAGE_RECEIVED;
	frame[1] = (sid & 0xff);
	frame[2] = (sid >> 8);
	frame[3] = length;

	for (i = 0; i < length; i++)
		frame[4 + i] = data[i];

	ringbufWrite(&out_rb, frame, frame_size);
}

void protocolMessageReceivedExt(uint32_t eid, const uint8_t* data, size_t length) {
	uint8_t frame[6 + 8];
	size_t frame_size = 6 + length;
	size_t i;

	frame[0] = DTH_MESSAGE_EXT_ID;
	frame[1] = (eid & 0xff);
	frame[2] = ((eid >> 8) & 0xff);
	frame[3] = ((eid >> 16) & 0xff);
	frame[4] = ((eid >> 24) & 0xff);
	frame[5] = length;

	for (i = 0; i < length; i++)
		frame[6 + i] = data[i];

	ringbufWrite(&out_rb, frame, frame_size);
}

void protocolSendHeartbeat(void) {
	ocarinaDataOut(heartbeat, sizeof(heartbeat));
}

void protocolProcess(void) {
	for (;;) {
		size_t readpos = in_rb.readpos;
		uint8_t cmd;

		if (!ringbufTryRead(&in_rb, &cmd, 1, &readpos))
			break;

		switch (cmd) {
		case CMD_DISABLE_RX:
		case CMD_ENABLE_RX:
			ocarinaRxEnabled = (cmd & 1);
			break;

		case CMD_QUERY_INTERFACE_ID:
		case '?': {
			uint8_t token = DTH_INTERFACE_ID;
			ringbufWrite(&out_rb, &token, 1)
					&& ringbufWrite(&out_rb, (const uint8_t*) s_interface_id, strlen(s_interface_id))
					&& ringbufWrite(&out_rb, (const uint8_t*) "\r\n", 2);
			break;
		}

		case CMD_QUERY_ERROR_FLAGS: {
			uint8_t token = DTH_ERROR_FLAGS;
			ringbufWrite(&out_rb, &token, 1)
					&& ringbufWrite(&out_rb, (const uint8_t*) &ocarinaErrorFlags, sizeof(ocarinaErrorFlags));
			break;
		}

		case CMD_QUERY_STATUS_FLAGS: {
			uint8_t token = DTH_STATUS_FLAGS;
			ringbufWrite(&out_rb, &token, 1)
					&& ringbufWrite(&out_rb, (const uint8_t*) &ocarinaStatusFlags, sizeof(ocarinaStatusFlags));
			break;
		}

		case CMD_SEND_MESSAGE_EXT_ID: {
			uint32_t eid;
			uint8_t length;
			uint8_t buffer[8];

			if (ringbufTryRead(&in_rb, (uint8_t*) &eid, sizeof(eid), &readpos) != sizeof(eid)
				|| ringbufTryRead(&in_rb, (uint8_t*) &length, sizeof(length), &readpos) != sizeof(length))
				return;

			/* Protocol error - try to recover */
			if (length > 8) {
				in_rb.readpos = readpos;
				ocarinaErrorFlags |= OCARINA_PROTOCOL_ERROR;
				return;
			}

			if (ringbufTryRead(&in_rb, buffer, length, &readpos) != length)
				return;

			ocarinaSendMessageExtId(eid, buffer, length);
			break;
		}

		case CMD_SEND_MESSAGE_STD_ID: {
			uint16_t sid;
			uint8_t length;
			uint8_t buffer[8];

			if (ringbufTryRead(&in_rb, (uint8_t*) &sid, sizeof(sid), &readpos) != sizeof(sid)
					|| ringbufTryRead(&in_rb, (uint8_t*) &length, sizeof(length), &readpos) != sizeof(length))
				return;

			/* Protocol error - try to recover */
			if (length > 8) {
				in_rb.readpos = readpos;
				ocarinaErrorFlags |= OCARINA_PROTOCOL_ERROR;
				return;
			}

			if (ringbufTryRead(&in_rb, buffer, length, &readpos) != length)
				return;

			ocarinaSendMessageStdId(sid, buffer, length);
			break;
		}

		case CMD_SET_BITRATE: {
			break;
		}

		case CMD_SYNC:
			ringbufWrite(&out_rb, sync, sizeof(sync));
			break;

		default:
			ocarinaErrorFlags |= OCARINA_PROTOCOL_ERROR;
		}

		in_rb.readpos = readpos;
	}

	// FIXME: check if usb_buf might not be in use
	size_t readpos = out_rb.readpos;
	size_t read = ringbufTryRead(&out_rb, usb_buf, sizeof(usb_buf), &readpos);

	if (read > 0) {
		// If sending fails (USB busy), keep the data in buffer
		if (ocarinaDataOut(usb_buf, read))
			out_rb.readpos = readpos;

	}
}

void ocarinaDataIn(const uint8_t* data, size_t length) {
	if (!ringbufWrite(&in_rb, data, length))
		ocarinaErrorFlags |= OCARINA_USBIN_OVERFLOW;
}
