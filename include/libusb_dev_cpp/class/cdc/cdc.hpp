#pragma once

#include <array>

#include <cstdint>
#include <cstddef>

enum class CDC_CLASS_CODE : uint8_t
{
	CDC = 0x02,
	CDC_DATA = 0x0A
};

enum class CDC_SUBCLASS_CODE : uint8_t
{
	ACM = 0x02
};

enum class CDC_PROTO_CODE : uint8_t
{
	V25TER = 0x02
};

enum class CDC_DATA_PROTO_CODE : uint8_t
{
	NTB     = 0x02,
	HOST    = 0xFD,
	CDCSPEC = 0xFE
};

enum class DTYPE_CDC : uint8_t
{
	HEADER    = 0x00,
	CALL_MGMT = 0x01,
	ACM       = 0x02,
	UNION     = 0x06,
	COUNTRY   = 0x07
};

enum class CDC_REQUESTS : uint8_t
{
	SEND_ENCAPSULATED_CMD  = 0x00,
	GET_ENCAPSULATED_RESP  = 0x01,
	SET_COMM_FEATURE       = 0x02,
	GET_COMM_FEATURE       = 0x03,
	CLEAR_COMM_FEATURE     = 0x04,
	SET_LINE_CODING        = 0x20,
	GET_LINE_CODING        = 0x21,
	SET_CONTROL_LINE_STATE = 0x22,
	SEND_BREAK             = 0x23
};

enum class CDC_NOTIFICATION : uint8_t
{
	NETWORK_CONNECTION = 0x00,
	RESPONSE_AVAILABLE = 0x01,
	SERIAL_STATE       = 0x20,
	SPEED_CHANGE       = 0x2A
};

enum class CDC_ACMGMNTCAP : uint8_t
{
	COMM_FEATURE = 0x01,
	LINE         = 0x02,
	BRK          = 0x04,
	NOTIFY       = 0x08
};

enum class CDC_CALLMGMTCAP : uint8_t
{
	CALL_MGMT = 0x01,
	DATA_INTF = 0x02
};

enum class CDC_LINECODE : uint8_t
{
	STOP_BITS_1   = 0x00,
	STOP_BITS_1_5 = 0x01,
	STOP_BITS_2   = 0x02,
	NO_PARITY     = 0x00,
	ODD_PARITY    = 0x01,
	EVEN_PARITY   = 0x02,
	MARK_PARITY   = 0x03,
	SPACE_PARITY  = 0x04
};

enum class CDC_STATE : uint16_t
{
	RX_CARRIER = 0x0001,
	TX_CARRIER = 0x0002,
	BREAK      = 0x0004,
	RING       = 0x0008,
	FRAMING    = 0x0010,
	PARITY     = 0x0020,
	OVERRUN    = 0x0040
};

class CDC_header_descriptor
{
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubType;
	uint8_t bcdCDC;
};

class CDC_call_management_descriptor
{
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubType;
	uint8_t bmCapabilities;
	uint8_t bDataInterface;
};

class CDC_acm_descriptor
{
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubType;
	uint8_t bmCapabilities;	
};

template<size_t DATALEN>
class CDC_notification
{
	uint16_t bmRequestType;
	uint8_t  bNotificationType;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	std::array<uint8_t, DATALEN> Data;
};

class CDC_line_coding
{
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
};
