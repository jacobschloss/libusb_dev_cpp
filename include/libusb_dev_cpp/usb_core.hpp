#pragma once

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

class USB_core
{
public:

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

	bool initialize(usb_driver_base* const driver, const uint8_t ep0size);
	bool poll();

	bool enable();
	bool disable();

	bool connect();
	bool disconnect();

	void config_ep();
	void deconfig_ep();

	int32_t ep_write();
	int32_t ep_read();
	int32_t ep_stall();
	int32_t ep_unstall();

protected:

	bool handle_event(const USB_common::USB_EVENTS evt, const uint8_t ep);

	bool handle_reset();

	bool handle_ep0_rx(const USB_common::USB_EVENTS event, const uint8_t ep);
	bool handle_ep0_tx(const USB_common::USB_EVENTS event, const uint8_t ep);
	bool handle_ep0_setup(const USB_common::USB_EVENTS event, const uint8_t ep);

	void handle_ep_rx(const uint8_t ep);
	void handle_ep_tx(const uint8_t ep);

	virtual USB_common::USB_RESP process_request(Control_request* const req);

	virtual USB_common::USB_RESP handle_device_request(Control_request* const req);
	virtual USB_common::USB_RESP handle_iface_request(Control_request* const req);
	virtual USB_common::USB_RESP handle_ep_request(Control_request* const req);

	virtual void handle_ctrl_req_complete();

	class usb_driver_buffer
	{
	public:
		usb_driver_buffer()
		{
			data_buf     = nullptr;
			data_ptr     = nullptr;
			data_count   = 0;
			data_maxsize = 0;
		}

		//start of buffer
		uint8_t* data_buf;

		//current position in buffer
		uint8_t* data_ptr;

		//requested length
		size_t   data_count;

		//max length
		size_t   data_maxsize;
	};

	usb_driver_base* m_driver;

	usb_driver_buffer m_rx_buffer;

	Control_request m_ctrl_req;
	std::array<uint8_t, 64> m_ctrl_req_data;
};