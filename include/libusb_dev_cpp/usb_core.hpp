#pragma once

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

#include "libusb_dev_cpp/core/Control_request.hpp"


#include <functional>

class USB_core
{
public:

	enum class USB_STATE
	{
		IDLE,
		RXDATA,
		TXDATA,
		TX_ZLP,
		LASTDATA,
		STATUS_IN,
		STATUS_OUT
	};

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

	enum class USB_CMD
	{
		ENABLE,
		DISABLE,
		CONNECT,
		DISCONNECT,
		RESET
	};

	enum class USB_RESP
	{
		FAIL,
		ACK,
		NAK
	};

	typedef std::function<void(USB_EVENTS event, uint8_t ep)> Event_callback;

	typedef std::function<bool(Control_request* ctrl_req)> Control_transfer_complete_callback;

	typedef std::function<bool(Control_request* ctrl_req, Control_transfer_complete_callback* callback)> Control_callback;

	typedef std::function<bool(Control_request* ctrl_req, uint8_t** address, size_t* size)> Get_descriptor_callback;

	typedef std::function<bool()> Set_configuration_callback;

	bool initialize(usb_driver_base* driver, const uint8_t ep0size);
	bool poll();

	bool enable();
	bool disable();

	bool connect();
	bool disconnect();

	void set_control_callback();
	void set_config_callback();
	void set_descriptor_callback();

	void set_ep_callback();
	void set_event_callback();

	void config_ep();
	void deconfig_ep();

	int32_t ep_write();
	int32_t ep_read();
	int32_t ep_stall();
	int32_t ep_unstall();

protected:

	usb_driver_base* m_driver;



};