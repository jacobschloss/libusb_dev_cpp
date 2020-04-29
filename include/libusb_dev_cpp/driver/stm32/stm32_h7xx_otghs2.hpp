/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019-2020 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

class stm32_h7xx_otghs2 : public usb_driver_base
{
public:

	stm32_h7xx_otghs2();
	~stm32_h7xx_otghs2() override;

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

	USB_common::USB_SPEED get_speed() const override;

	void poll(const USB_common::Event_callback& func) override;

	const ep_cfg& get_ep0_config() const override;
	bool get_rx_ep_config(const uint8_t addr, ep_cfg* const out_ep) override;
	bool get_tx_ep_config(const uint8_t addr, ep_cfg* const out_ep) override;

	const Setup_packet::Setup_packet_array* get_last_setup_packet() const override
	{
		return &m_last_setup_packet;
	}

	//application waits for a buffer with data
	//this might be better as a stream thing rather than buffer exchange
	Buffer_adapter_base* wait_rx_buffer(const uint8_t ep) override;
	//application returns rx buffer to driver. will allow reception to continue in event of buffer underrun
	void release_rx_buffer(const uint8_t ep, Buffer_adapter_base* const buf) override;

	//application wait for usable tx buffer
	Buffer_adapter_base* wait_tx_buffer(const uint8_t ep) override;
	//application give buffer to driver for transmission
	bool enqueue_tx_buffer(const uint8_t ep, Buffer_adapter_base* const buf) override;

	size_t get_serial_number(uint8_t* const buf, const size_t maxlen)
	{
		return 0;
	}

protected:

	bool handle_reset_done();
	bool handle_enum_done();

	enum class STATE
	{
		UNKNOWN,
		DISCONNECTED,
		ATTATCHED,
		WAIT_RESET,
		RESET_DET,
		ENUM_DONE
	};

	STATE m_state;

	void set_data0(const uint8_t ep) override;

	bool handle_iepintx(const USB_common::Event_callback& func);
	bool handle_oepintx(const USB_common::Event_callback& func);

	void flush_rx();
	void flush_tx(const uint8_t ep);
	void flush_all_tx();

	void core_reset();

	static bool config_ep_tx_fifo(const uint8_t ep, const size_t len);

	static constexpr size_t MAX_NUM_EP = 8;//ep0 + ep1..ep8
	static constexpr size_t MAX_RX_PACKET = 512;
	static constexpr size_t RX_FIFO_SIZE = (5*1+8) + 2*(MAX_RX_PACKET/4+1) + (2*9) + 1;//maybe use 1280?
	static constexpr size_t MAX_FIFO_LEN_U32 = 1024; //uint32 * 1024, 4096B
	static constexpr size_t MAX_FIFO_LEN_U8  = 4096; //uint8  * 4096, 4096B

	ep_cfg m_ep0_cfg;
	std::array<ep_cfg, MAX_NUM_EP> m_rx_ep_cfg;
	std::array<ep_cfg, MAX_NUM_EP> m_tx_ep_cfg;

	Setup_packet::Setup_packet_array m_last_setup_packet;
};
