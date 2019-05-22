#pragma once

#include "libusb_dev_cpp/core/Request_type.hpp"

#include <array>

#include <cstdint>

class Setup_packet
{
public:

	typedef std::array<uint8_t, 8> Setup_packet_array;

	enum class DEVICE_REQUEST : uint8_t
	{
		GET_STATUS        = 0x00,
		CLEAR_FEATURE     = 0x01,
		SET_FEATURE       = 0x03,
		SET_ADDRESS       = 0x05,
		GET_DESCRIPTOR    = 0x06,
		SET_DESCRIPTOR    = 0x07,
		GET_CONFIGURATION = 0x08,
		SET_CONFIGURATION = 0x09
	};

	enum class INTERFACE_REQUEST : uint8_t
	{
		GET_STATUS    = 0x00,
		CLEAR_FEATURE = 0x01,
		SET_FEATURE   = 0x03,
		GET_INTERFACE = 0x0A,
		SET_INTERFACE = 0x11
	};

	enum class ENDPOINT_REQUEST : uint8_t
	{
		GET_STATUS    = 0x00,
		CLEAR_FEATURE = 0x01,
		SET_FEATURE   = 0x03,
		SYNC_FRAME    = 0x0A
	};

	bool serialize(Setup_packet_array* const out_array);
	bool deserialize(const Setup_packet_array& in_array);

	bool get_request_type(Request_type* const request_type);

	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
protected:
};

