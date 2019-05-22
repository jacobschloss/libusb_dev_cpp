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
		SUSPEND,
		WAKEUP,
		EPRX,
		EPTX,
		EPSETUP,
		ERROR
	};
	static constexpr size_t USB_EVENTS_MAX = 8;

	typedef std::function<void(const USB_EVENTS event, const uint8_t ep)> Event_callback;
	typedef std::function<bool(Control_request* ctrl_req)> Control_transfer_complete_callback;
	typedef std::function<bool(Control_request* ctrl_req, Control_transfer_complete_callback* callback)> Control_callback;
	typedef std::function<bool(Control_request* ctrl_req, uint8_t** address, size_t* size)> Get_descriptor_callback;
	typedef std::function<bool()> Set_configuration_callback;

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
