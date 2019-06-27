/**
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "common_util/Byte_util.hpp"

#include <functional>

#include <cstdint>

class USB_common
{
public:

	enum class USB_EVENTS
	{
		RESET,
		SOF,
		EARLY_SUSPEND,
		SUSPEND,
		WAKEUP,
		EP_RX,
		EP_TX,
		CTRL_SETUP_PHASE_DONE,
		CTRL_STATUS_PHASE,
		NONE
	};
	static constexpr size_t USB_EVENTS_MAX = 8;

	enum class USB_RESP
	{
		FAIL,
		ACK,
		NAK
	};

	enum class USB_SPEED
	{
		LS,
		FS,
		HS
	};

	enum class DESCRIPTOR_TYPE
	{
		DEVICE                        = 0x01,
		CONFIGURATION                 = 0x02,
		STRING                        = 0x03,
		INTERFACE                     = 0x04,
		ENDPOINT                      = 0x05,
		DEVICE_QUALIFIER              = 0x06,
		OTHER_SPEED_CONFIGURATION     = 0x07,
		INTERFACE_POWER               = 0x08,
		OTG                           = 0x09,
		DEBUG                         = 0x0A,
		INTERFASE_ASSOCIATION         = 0x0B,
		CLASS_SPECIFIC_INTERFACE      = 0x24,
		CLASS_SPECIFIC_ENDPOINT       = 0x25
	};

	enum class CLASS_DEF
	{
		CLASS_PER_INTERFACE = 0x00
	};

	enum class SUBCLASS_DEF
	{
		SUBCLASS_NONE = 0x00
	};

	enum class PROTO_DEF
	{
		PROTO_NONE = 0x00
	};

	typedef std::function<void(const USB_EVENTS event, const uint8_t ep)> Event_callback;

	static bool is_in_ep(const uint8_t ep)
	{
		return (ep & 0x80) != 0;
	}

	static bool is_out_ep(const uint8_t ep)
	{
		return (ep & 0x80) == 0;
	}

	static bool get_ep_addr(const uint8_t ep)
	{
		return ep & 0x7F;
	}

	static constexpr uint16_t build_bcd(const uint8_t major, const uint8_t minor, const uint8_t subminor)
	{
		//0xJJMN
		return Byte_util::make_u16(major, ((minor & 0x0F) << 4) | (subminor & 0x0F));
	}
};
