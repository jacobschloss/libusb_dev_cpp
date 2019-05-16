#pragma once

#include <cstdint>
#include <cstddef>

class usb_driver_base
{
public:

	usb_driver_base();
	virtual ~usb_driver_base();

	virtual bool initialize() = 0;

	virtual void get_info() = 0;

	virtual bool enable() = 0;
	virtual bool disable() = 0;

	virtual bool connect() = 0;
	virtual bool disconnect() = 0;

	virtual bool set_usb_address(const uint8_t addr) = 0;

	virtual size_t ep_setup(const uint8_t ep) = 0;

	virtual size_t ep_config(const uint8_t ep) = 0;
	virtual size_t ep_unconfig(const uint8_t ep) = 0;

	virtual bool ep_is_stalled(const uint8_t ep) = 0;
	virtual void ep_stall(const uint8_t ep) = 0;
	virtual void ep_unstall(const uint8_t ep) = 0;

	virtual size_t ep_write(const uint8_t ep, const uint8_t* buf, const uint16_t len) = 0;
	virtual size_t ep_read(const uint8_t ep, uint8_t* const buf, const uint16_t len) = 0;

	virtual uint16_t get_frame_number() = 0;
	virtual size_t get_serial_number(uint8_t* const buf, const size_t maxlen) = 0;

protected:

};