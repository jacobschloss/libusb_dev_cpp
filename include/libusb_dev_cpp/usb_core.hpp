#pragma once

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

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

	bool handle_event(const USB_common::USB_EVENTS evt, const uint8_t ep);

	bool handle_ep0_rx(const USB_common::USB_EVENTS event, const uint8_t ep);
	bool handle_ep0_tx(const USB_common::USB_EVENTS event, const uint8_t ep);
	bool handle_ep0_setup(const USB_common::USB_EVENTS event, const uint8_t ep);

	usb_driver_base* m_driver;
};