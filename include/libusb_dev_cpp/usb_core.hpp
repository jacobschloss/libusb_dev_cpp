#pragma once

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

class USB_core
{
public:



	bool initialize(usb_driver_base* driver, const uint8_t ep0size);
	bool poll();

	void enable();
	void disable();

	void connect();

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