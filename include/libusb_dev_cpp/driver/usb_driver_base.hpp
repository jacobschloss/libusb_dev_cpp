#pragma once

#include <cstdint>
#include <cstddef>

class usb_driver_base
{
public:

	usb_driver_base();
	virtual ~usb_driver_base();

	virtual bool initialize() = 0;
	virtual bool connect() = 0;
	virtual bool disconnect() = 0;

	virtual bool set_address() = 0;

	virtual size_t setup_ep(uint8_t ep) = 0;

	virtual size_t config_ep(uint8_t ep) = 0;
	virtual size_t unconfig_ep(uint8_t ep) = 0;

	virtual void stall_ep(uint8_t ep) = 0;
	virtual void unstall_ep(uint8_t ep) = 0;

	virtual size_t write_ep(uint8_t ep) = 0;
	virtual size_t read_ep(uint8_t ep) = 0;

protected:

};