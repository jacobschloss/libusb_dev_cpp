/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

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
		SET_CONFIGURATION = 0x09,
		SET_SEL           = 0x30
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

	// USB 3.2 Revision 1.0 Table 9-7
	enum class FEATURE_SELECTOR_ENDPOINT : uint8_t
	{
		ENDPOINT_HALT = 0,
	};

	// USB 3.2 Revision 1.0 Table 9-7
	enum class FEATURE_SELECTOR_INTERFACE : uint8_t
	{
		FUNCTION_SUSPEND = 0,
	};

	// USB 3.2 Revision 1.0 Table 9-7
	enum class FEATURE_SELECTOR_DEVICE : uint8_t
	{
		DEVICE_REMOTE_WAKEUP = 1,
		TEST_MODE            = 2,
		b_hnp_enable         = 3,
		a_hnp_support        = 4,
		a_alt_hnp_support    = 5,
		WUSB_DEVICE          = 6,
		U1_ENABLE            = 48,
		U2_ENABLE            = 49,
		LTM_ENABLE           = 50,
		B3_NTF_HOST_REL      = 51, // Reserved for OTG
		B3_RSP_ENABLE        = 52, // Reserved for OTG
		LDM_ENABLE           = 53
	};

	// USB 2.0 Table 9-7 Test Mode Selectors
	enum class TEST_MODE_SELECTOR
	{
		TEST_J            = 0x01,
		TEST_K            = 0x02,
		TEST_SE0_NAK      = 0x03,
		TEST_PACKET       = 0x04,
		TEST_FORCE_ENABLE = 0x05
	};

	// USB 3.2 Revision 1.0 Table 9-8
	enum class STANDARD_STATUS_TYPE : uint8_t
	{
		STANDARD_STATUS = 0x00,
		PTM_STATUS      = 0x01
	};

	bool serialize(Setup_packet_array* const out_array);
	bool deserialize(const Setup_packet_array& in_array);

	bool get_request_type(Request_type* const request_type) const;

	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
protected:
};

