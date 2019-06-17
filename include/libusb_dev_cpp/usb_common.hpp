#pragma once

#include "libusb_dev_cpp/core/Control_request.hpp"

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
		DEVICE                    = 0x01,
		CONFIGURATION             = 0x02,
		STRING                    = 0x03,
		INTERFACE                 = 0x04,
		ENDPOINT                  = 0x05,
		DEVICE_QUALIFIER          = 0x06,
		OTHER_SPEED_CONFIGURATION = 0x07
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
};
