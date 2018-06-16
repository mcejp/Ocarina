
/* host -> device */
enum {
    CMD_QUERY_INTERFACE_ID =    0x01,
    CMD_SEND_MESSAGE_STD_ID =   0x02,
    CMD_QUERY_ERROR_FLAGS =     'e',
    CMD_QUERY_STATUS_FLAGS =    's',
    //CMD_SEND_MESSAGE_STD_ID = 't',
    CMD_SET_BITRATE =           'S',
    CMD_SEND_MESSAGE_EXT_ID =   'T',
    CMD_DISABLE_RX =            0xF0,
    CMD_ENABLE_RX =             0xF1,
    CMD_SYNC =                  0xAA,
};

enum {
    BITRATE_10kbit =    0,
    BITRATE_20kbit =    1,
    BITRATE_50kbit =    2,
    BITRATE_100kbit =   3,
    BITRATE_125kbit =   4,
    BITRATE_250kbit =   5,
    BITRATE_500kbit =   6,
    BITRATE_800kbit =   7,
    BITRATE_1Mbit =     8,
};

/* device -> host */
enum {
    DTH_INTERFACE_ID =      0x01,
    DTH_MESSAGE_RECEIVED =  0x02,
    DTH_ERROR_FLAGS =       'e',
    DTH_MESSAGE_STD_ID =    'r',
    DTH_STATUS_FLAGS =      's',
    DTH_MESSAGE_EXT_ID =    'R',
};

enum {
	OCARINA_USBIN_OVERFLOW = (1<<0),
	//OCARINA_USBOUT_OVERFLOW = (1<<1),
	OCARINA_CANIN_OVERFLOW = (1<<2),
	//OCARINA_CANOUT_OVERFLOW = (1<<3),
	OCARINA_PROTOCOL_ERROR = (1<<4),
};

// ocarina.c -> protocol.c

void protocolInit(const char* interface_id);
void protocolProcess(void);

void protocolSendHeartbeat(void);	/* TODO: self-handle */
void protocolMessageReceivedExt(uint32_t eid, const uint8_t* data, size_t length);
void protocolMessageReceivedStd(uint16_t sid, const uint8_t* data, size_t length);

// protocol.c -> ocarina.c

int ocarinaSendMessageExtId(uint32_t eid, const void* data, size_t length);
int ocarinaSendMessageStdId(uint16_t sid, const void* data, size_t length);
void ocarinaFeaturesEnable();
