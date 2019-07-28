/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/core/Setup_packet.hpp"

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"
#include "libusb_dev_cpp/util/EP_buffer_mgr_base.hpp"

#include <cstdint>
#include <cstddef>

class usb_driver_base
{
public:

	enum class USB_SPEED
	{
		LS,
		FS,
		HS
	};

	enum class EP_TYPE
	{
		CONTROL,
		ISOCHRONUS,
		INTERRUPT,
		BULK,
		UNCONF
	};

	struct ep_cfg
	{
		uint8_t num;
		size_t  size;
		EP_TYPE type;
	};

	struct Data_packet
	{
		std::array<uint8_t, 512> buf;
		size_t len;

		uint8_t* data()
		{
			return buf.data();
		}

		const uint8_t* data() const
		{
			return buf.data();
		}

		size_t size() const
		{
			return len;
		}
	};

	static size_t get_max_bulk_ep_size(const USB_SPEED& speed)
	{
		size_t size = 0;
		switch(speed)
		{
			case USB_SPEED::LS:
			{
				size = 64;
			}
			case USB_SPEED::FS:
			{
				size = 64;	
			}
			case USB_SPEED::HS:
			{
				size = 512;
			}
			default:
			{
				break;
			}
		}

		return size;
	};

	usb_driver_base();
	virtual ~usb_driver_base() = default;

	virtual bool initialize() = 0;

	virtual void get_info() = 0;

	virtual bool enable() = 0;
	virtual bool disable() = 0;

	virtual bool connect() = 0;
	virtual bool disconnect() = 0;

	virtual bool set_address(const uint8_t addr) = 0;

	virtual bool ep_config(const ep_cfg& ep) = 0;
	virtual bool ep_unconfig(const uint8_t ep) = 0;

	virtual bool ep_is_stalled(const uint8_t ep) = 0;
	virtual void ep_stall(const uint8_t ep) = 0;
	virtual void ep_unstall(const uint8_t ep) = 0;

	virtual int ep_write(const uint8_t ep, const uint8_t* buf, const uint16_t len) = 0;
	virtual int ep_read(const uint8_t ep, uint8_t* const buf, const uint16_t max_len) = 0;

	virtual uint16_t get_frame_number() = 0;
	virtual size_t get_serial_number(uint8_t* const buf, const size_t maxlen) = 0;

	virtual USB_common::USB_SPEED get_speed() const = 0;

	bool set_ep_rx_callback(const uint8_t ep, const USB_common::Event_callback& func);
	bool set_ep_tx_callback(const uint8_t ep, const USB_common::Event_callback& func);
	bool set_ep_setup_callback(const uint8_t ep, const USB_common::Event_callback& func);

	const USB_common::Event_callback& get_event_callback(const uint8_t ep_addr) const
	{
		return m_event_callbacks[ep_addr];
	}

	const USB_common::Event_callback& get_ep_rx_callback(const uint8_t ep_addr) const
	{
		return m_ep_rx_callbacks[ep_addr];
	}

	const USB_common::Event_callback& get_ep_tx_callback(const uint8_t ep_addr) const
	{
		return m_ep_tx_callbacks[ep_addr];
	}

	const USB_common::Event_callback& get_ep_setup_callback(const uint8_t ep_addr) const
	{
		return m_ep_setup_callbacks[ep_addr];
	}

	virtual void poll(const USB_common::Event_callback& func) = 0;

	virtual const ep_cfg& get_ep0_config() const = 0;
	virtual bool get_rx_ep_config(const uint8_t addr, ep_cfg* const out_ep) = 0;
	virtual bool get_tx_ep_config(const uint8_t addr, ep_cfg* const out_ep) = 0;

	virtual void set_data0(const uint8_t ep) = 0;

	virtual const Setup_packet::Setup_packet_array* get_last_setup_packet() const = 0;

	void set_ep0_buffer(EP_buffer_mgr_base* const ep0_buffer)
	{
		m_ep0_buffer = ep0_buffer;
	}

	EP_buffer_mgr_base* get_ep0_buffer()
	{
		return m_ep0_buffer;
	}

	void set_tx_buffer(EP_buffer_mgr_base* const tx_buffer)
	{
		m_tx_buffer = tx_buffer;
	}

	EP_buffer_mgr_base* get_tx_buffer()
	{
		return m_tx_buffer;
	}

	void set_rx_buffer(EP_buffer_mgr_base* const rx_buffer)
	{
		m_rx_buffer = rx_buffer;
	}

	EP_buffer_mgr_base* get_rx_buffer()
	{
		return m_rx_buffer;
	}

	//application waits for a buffer with data
	//this might be better as a stream thing rather than buffer exchange
	virtual Buffer_adapter_base* wait_rx_buffer(const uint8_t ep) = 0;
	//application returns rx buffer to driver. will allow reception to continue in event of buffer underrun
	virtual void release_rx_buffer(const uint8_t ep, Buffer_adapter_base* const buf) = 0;

	//application wait for usable tx buffer
	virtual Buffer_adapter_base* wait_tx_buffer(const uint8_t ep) = 0;
	//application give buffer to driver for transmission
	virtual bool enqueue_tx_buffer(const uint8_t ep, Buffer_adapter_base* const buf) = 0;

protected:

	EP_buffer_mgr_base* m_ep0_buffer;
	EP_buffer_mgr_base* m_tx_buffer;
	EP_buffer_mgr_base* m_rx_buffer;

	std::array<USB_common::Event_callback, USB_common::USB_EVENTS_MAX> m_event_callbacks;
	std::array<USB_common::Event_callback, 8>                          m_ep_rx_callbacks;
	std::array<USB_common::Event_callback, 8>                          m_ep_tx_callbacks;
	std::array<USB_common::Event_callback, 8>                          m_ep_setup_callbacks;
};