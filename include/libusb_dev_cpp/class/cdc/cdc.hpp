/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include <array>

#include <cstdint>
#include <cstddef>

namespace CDC
{

	constexpr static uint8_t COMM_DEVICE_CLASS_CODE    = 0x02;
	constexpr static uint8_t COMM_INTERFACE_CLASS_CODE = 0x02;

	enum class COMM_CLASS_SUBCLASS_CODE : uint8_t
	{
		DLCM   = 0x01, // USBPSTN
		ACM    = 0x02, // USBPSTN
		TCM    = 0x03, // USBPSTN
		MCCM   = 0x04, // USBISDN
		CAPICM = 0x05, // USBISDN
		ENCM   = 0x06, // USBECM
		ATM    = 0x07, // USBATM
		WHCM   = 0x08, // USBWMC
		DM     = 0x09, // USBWMC
		MDLM   = 0x0A, // USBWMC
		OBEX   = 0x0B, // USBWMC
		EEM    = 0x0C, // USBEEM
		NCM    = 0x0D  // USBNCM
	};

	enum class COMM_CLASS_PROTO_CODE : uint8_t
	{
		NONE      = 0x00,
		V250      = 0X01,
		PCCA101   = 0x02,
		PCCA101AO = 0x03,
		GSM707    = 0x04,
		GPP2707   = 0x05,
		CS0017O   = 0x06,
		USBEEM    = 0x07,
		VENDOR    = 0xFF
	};

	constexpr static uint8_t DATA_INTERFACE_CLASS_CODE    = 0x0A;
	constexpr static uint8_t DATA_INTERFACE_SUBCLASS_CODE = 0x00;

	enum class DATA_INTERFACE_PROTO_CODE : uint8_t
	{
		NONE    = 0x00,
		NTB     = 0x01,
		HOST    = 0xFD,
		VENDOR  = 0xFF
	};

	enum class FUNC_DESCRIPTOR_TYPE : uint8_t
	{
		HEADER    = 0x00,
		CALL_MGMT = 0x01,
		ACM       = 0x02,
		UNION     = 0x06,
		COUNTRY   = 0x07
	};

	enum class FUNC_DESCRIPTOR_SUBTYPE : uint8_t
	{
		HEADER    = 0x00
	};

	enum class CDC_REQUESTS : uint8_t
	{
		SEND_ENCAPSULATED_CMD  = 0x00,
		GET_ENCAPSULATED_RESP  = 0x01,
		SET_COMM_FEATURE       = 0x02,
		GET_COMM_FEATURE       = 0x03,

		SET_AUX_LINE_STATE     = 0x10,
		SET_HOOK_STATE         = 0x11,
		PULSE_SETUP            = 0x12,
		SEND_PULSE             = 0x13,
		SET_PULSE_TIME         = 0x14,
		RING_AUX_JACK          = 0x15,

		CLEAR_COMM_FEATURE     = 0x04,
		SET_LINE_CODING        = 0x20,
		GET_LINE_CODING        = 0x21,
		SET_CONTROL_LINE_STATE = 0x22,
		SEND_BREAK             = 0x23,

		SET_RINGER_PARMS       = 0x30,
		GET_RINGER_PARMS       = 0x31,
		SET_OPERATION_PARMS    = 0x32,
		GET_OPERATION_PARMS    = 0x33,

		SET_LINE_PARMS         = 0x34,
		GET_LINE_PARMS         = 0x35,
		DIAL_DIGITS            = 0x36,
		SET_UNIT_PARAMETER     = 0x37,
		GET_UNIT_PARAMETER     = 0x38,
		CLEAR_UNIT_PARAMETER   = 0x39,
		GET_PROFILE            = 0x3A,

		SET_ETHERNET_MULTICAST_FILTERS               = 0x40,
		SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER = 0x41,
		GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER = 0x42,
		SET_ETHERNET_PACKET_FILTER                   = 0x43,
		GET_ETHERNET_STATISTIC                       = 0x44,

		SET_ATM_DATA_FORMAT                          = 0x50,
		GET_ATM_DEVICE_STATISTICS                    = 0x51,
		SET_ATM_DEFAULT_VC                           = 0x52,
		GET_ATM_VC_STATISTICS                        = 0x53,

		GET_NTB_PARAMETERS    = 0x80,
		GET_NET_ADDRESS       = 0x81,
		SET_NET_ADDRESS       = 0x82,
		GET_NTB_FORMAT        = 0x83,
		SET_NTB_FORMAT        = 0x84,
		GET_NTB_INPUT_SIZE    = 0x85,
		SET_NTB_INPUT_SIZE    = 0x86,
		GET_MAX_DATAGRAM_SIZE = 0x87,
		SET_MAX_DATAGRAM_SIZE = 0x88,
		GET_CRC_MODE          = 0x89,
		SET_CRC_MODE          = 0x8A
	};

	enum class CDC_NOTIFICATION : uint8_t
	{
		NETWORK_CONNECTION      = 0x00,
		RESPONSE_AVAILABLE      = 0x01,
		AUX_JACK_HOOK_STATE     = 0x08,
		RING_DETECT             = 0x09,
		SERIAL_STATE            = 0x20,
		CALL_STATE_CHANGE       = 0x28,
		LINE_STATE_CHANGE       = 0x29,
		CONNECTION_SPEED_CHANGE = 0x2A
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
}
