#pragma once

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

class stm32_h7xx_otghs : public usb_driver_base
{
public:

	stm32_h7xx_otghs();
	~stm32_h7xx_otghs() override;

	bool initialize() override;

	void get_info() override;

	bool enable() override;
	bool disable() override;

	bool connect() override;
	bool disconnect() override;

	bool set_usb_address(const uint8_t addr) override;

	size_t ep_setup(const uint8_t ep) override;

	size_t ep_config(const uint8_t ep) override;
	size_t ep_unconfig(const uint8_t ep) override;

	bool ep_is_stalled(const uint8_t ep) override;
	void ep_stall(const uint8_t ep) override;
	void ep_unstall(const uint8_t ep) override;

	size_t ep_write(const uint8_t ep, const uint8_t* buf, const uint16_t len) override;
	size_t ep_read(const uint8_t ep, uint8_t* const buf, const uint16_t len) override;

	uint16_t get_frame_number() override;
	size_t get_serial_number(uint8_t* const buf, const size_t maxlen) override;

protected:
};