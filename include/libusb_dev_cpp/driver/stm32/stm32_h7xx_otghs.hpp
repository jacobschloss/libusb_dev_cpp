/**
 * @brief stm32_h7xx_otghs
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

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

	bool set_address(const uint8_t addr) override;

	bool ep_config(const ep_cfg& ep) override;
	bool ep_unconfig(const uint8_t ep) override;

	bool ep_is_stalled(const uint8_t ep) override;
	void ep_stall(const uint8_t ep) override;
	void ep_unstall(const uint8_t ep) override;

	int ep_write(const uint8_t ep, const uint8_t* buf, const uint16_t len) override;
	int ep_read(const uint8_t ep, uint8_t* const buf, const uint16_t max_len) override;

	uint16_t get_frame_number() override;
	size_t get_serial_number(uint8_t* const buf, const size_t maxlen) override;

	USB_common::USB_SPEED get_speed() const override;

	void poll(const USB_common::Event_callback& func) override;

	const ep_cfg& get_ep0_config() const override;
	bool get_rx_ep_config(const uint8_t addr, ep_cfg* const out_ep) override;
	bool get_tx_ep_config(const uint8_t addr, ep_cfg* const out_ep) override;

protected:

	void flush_rx();
	void flush_tx(const uint8_t ep);
	void flush_all_tx();

	void core_reset();

	static bool config_ep_tx_fifo(const uint8_t ep, const size_t len);

	static constexpr size_t MAX_NUM_EP = 5;//ep0 + ep1..ep5
	static constexpr size_t MAX_RX_PACKET = 512;
	static constexpr size_t RX_FIFO_SIZE = (10 + (2*MAX_RX_PACKET/4) + 2 + 2);
	static constexpr size_t MAX_FIFO_LEN_U32 = 1024; //uint32 * 1024, 4096B
	static constexpr size_t MAX_FIFO_LEN_U8  = 4096; //uint8  * 4096, 4096B

	ep_cfg m_ep0_cfg;
	std::array<ep_cfg, MAX_NUM_EP> m_rx_ep_cfg;
	std::array<ep_cfg, MAX_NUM_EP> m_tx_ep_cfg;
};