#pragma once

enum ServiceChannelNotification : int {
	NO_SERVICE_EVENT = 0x01,
	MAP_CHANNEL_FRAME_BUFFER_FULL = 0x02,
	MASTER_CHANNEL_FRAME_BUFFER_FULL = 0x03,
	VC_MC_FRAME_BUFFER_FULL = 0x04,
	TX_MC_FRAME_BUFFER_FULL = 0x05,
	NO_TX_PACKETS_TO_PROCESS = 0x06,
	NO_RX_PACKETS_TO_PROCESS = 0x07,
	PACKET_EXCEEDS_MAX_SIZE = 0x08,
	FOP_SENT_QUEUE_FULL = 0x09,
	TX_TO_BE_TRANSMITTED_FRAMES_LIST_EMPTY = 0x0A,
	TX_TO_BE_TRANSMITTED_FRAMES_LIST_FULL = 0x0B,
	FOP_REQUEST_REJECTED = 0x0C,
	RX_IN_MC_FULL = 0x0D,
	RX_IN_BUFFER_FULL = 0x0E,
	RX_OUT_BUFFER_FULL = 0x0F,
	RX_INVALID_TFVN = 0x10,
	RX_INVALID_SCID = 0x11,
	RX_INVALID_LENGTH = 0x12,
	VC_RX_WAIT_QUEUE_FULL = 0x13,
	TX_FOP_REJECTED = 0x14,
	VC_MC_FRAME_BUFFER_EMPTY = 0x15,
	INVALID_VC_ID = 0x16,
	INVALID_MAP_ID = 0x17,
	VC_RECEPTION_BUFFER_AFTER_FARM_FULL = 0x18,
	NO_PACKETS_TO_PROCESS_IN_VC_RECEPTION_BEFORE_FARM = 0x19,
	RX_INVALID_CRC = 0x1A,
	INVALID_SERVICE_CALL = 0x1B,
    PACKET_BUFFER_EMPTY = 0x1C,
    NO_TX_PACKETS_TO_TRANSFER_FRAME = 0x1D,
    MC_RX_INVALID_COUNT = 0x1E,
};

enum SynchronizationFlag : bool { OCTET = 0, FORWARD_ORDERED = 1 };

enum FOPNotification : uint8_t {
	NO_FOP_EVENT = 0x01,
	SENT_QUEUE_FULL = 0x02,
	WAIT_QUEUE_EMPTY = 0x03,
};

enum COPDirectiveResponse : uint8_t {
	ACCEPT = 0x01,
	REJECT = 0x02,
};

enum VirtualChannelAlert : uint8_t {
	NO_VC_ALERT = 0x01,
	UNPROCESSED_PACKET_LIST_FULL = 0x02,
	TX_WAIT_QUEUE_FULL = 0x03,
	RX_WAIT_QUEUE_FULL = 0x04,
};

enum MasterChannelAlert : uint8_t {
	NO_MC_ALERT = 0x01,
	OUT_FRAMES_LIST_FULL = 0x02,
	TO_BE_TRANSMITTED_FRAMES_LIST_FULL = 0x03,
	MAX_AMOUNT_OF_VIRT_CHANNELS = 0x04,
	NO_SPACE = 0x05
};

enum TxRx : uint8_t { Rx = 0x00, Tx = 0x01 };

enum NotificationType : uint8_t {

	TypeVirtualChannelAlert = 0x00,
	TypeMasterChannelAlert = 0x01,
	TypeServiceChannelNotif = 0x02,
	TypeCOPDirectiveResponse = 0x03,
	TypeFOPNotif = 0x04,
	TypeFDURequestType = 0x05

};
