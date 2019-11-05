/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/usb_common.hpp"
#include "libusb_dev_cpp/core/Setup_packet.hpp"

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

#include "libusb_dev_cpp/util/Descriptor_table.hpp"
#include "libusb_dev_cpp/util/Buffer_adapter.hpp"
#include "libusb_dev_cpp/util/EP_buffer_mgr_base.hpp"

#include "freertos_cpp_util/Queue_static_pod.hpp"

class USB_core
{
public:

	typedef std::function<bool (void*, const uint16_t)> SetConfigurationCallback;

	enum class USB_CMD
	{
		ENABLE,
		DISABLE,
		CONNECT,
		DISCONNECT,
		RESET
	};

	USB_core();
	virtual ~USB_core();

	//no copy
	USB_core(const USB_core& rhs) = delete;
	USB_core& operator=(const USB_core& rhs) = delete;

	bool initialize(usb_driver_base* const driver, const uint8_t ep0size, const Buffer_adapter& tx_buf, const Buffer_adapter& rx_buf);
	void set_descriptor_table(Descriptor_table* const desc_table);
	void set_config_callback(const SetConfigurationCallback& callback, void* ctx)
	{
		m_set_config_callback_func = callback;
		m_set_config_callback_ctx = ctx;
	}
	
	//poll driver
	bool poll_driver();

	//handle events
	bool poll_event_loop();
	bool poll_event_loop(const bool wait);

	bool wait_event_loop();

	bool enable();
	bool disable();

	bool connect();
	bool disconnect();

	usb_driver_base* get_driver()
	{
		return m_driver;
	}

protected:

	bool handle_event(const USB_common::USB_EVENTS evt, const uint8_t ep);

	bool handle_reset();
	bool handle_enum_done();
	bool handle_sof();

	bool handle_ep0_setup(const USB_common::USB_EVENTS event, const uint8_t ep);
	bool handle_ep0_rx(const USB_common::USB_EVENTS event, const uint8_t ep);
	bool handle_ep0_tx(const USB_common::USB_EVENTS event, const uint8_t ep);

	virtual USB_common::USB_RESP process_request(Setup_packet* const req);

	virtual USB_common::USB_RESP handle_std_device_request(Setup_packet* const req);
	virtual USB_common::USB_RESP handle_std_iface_request(Setup_packet* const req);
	virtual USB_common::USB_RESP handle_std_ep_request(Setup_packet* const req);

	virtual bool set_configuration(const uint8_t bConfigurationValue);
	virtual bool get_configuration(uint8_t* const bConfigurationValue);

	virtual void handle_ctrl_req_complete();

	void stall_control_ep(const uint8_t ep);

	void set_address(const uint8_t addr);

	Buffer_adapter m_rx_buffer;
	Buffer_adapter m_tx_buffer;

	struct usb_core_event
	{
		USB_common::USB_EVENTS event;
		uint8_t	ep;
	};
	Queue_static_pod<usb_core_event, 32> m_event_queue;

/*
	enum class USB_DEVICE_STATE
	{
		DEFAULT,
		ADDRESS,
		CONFIGURED,
		SUSPENDED
	};
	USB_DEVICE_STATE m_device_state;
*/
	enum class USB_CONTROL_STATE
	{
		//no control traffic
		IDLE,
		//waiting to rx control trafic
		RXDATA,
		//waiting to send control trafic
		TXDATA,
		//waiting to send zlp
		TXZLP,
		//waiting for zlp to finish
		TXCOMP,
		//status in
		STATUS_IN,
		//status out
		STATUS_OUT
	};
	USB_CONTROL_STATE m_control_state;

	std::function<void()> m_setup_complete_callback;

	usb_driver_base* m_driver;

	Setup_packet m_setup_packet;

	uint8_t m_address;
	uint8_t m_configuration;

	//user data
	Descriptor_table* m_desc_table;

	void* m_set_config_callback_ctx;
	SetConfigurationCallback m_set_config_callback_func;

	USB_common::Event_callback m_usb_core_handle_event;
};