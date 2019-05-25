#pragma once

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

class USB_core
{
public:

	class buffer_adapter
	{
	public:
		buffer_adapter()
		{
			clear();
		}

		void clear()
		{
			buf_ptr     = nullptr;
			buf_maxsize = 0;
			curr_ptr    = nullptr;
			rem_len     = 0;
		}

		void reset()
		{
			curr_ptr    = buf_ptr;
			rem_len     = 0;	
		}

		void reset(uint8_t* const buf, const size_t maxlen)
		{
			buf_ptr     = buf;
			buf_maxsize = maxlen;
			curr_ptr    = buf;
			rem_len     = 0;	
		}

		//start of buffer
		uint8_t* buf_ptr;

		//max length
		size_t   buf_maxsize;

		//current position in buffer
		uint8_t* curr_ptr;

		//remaining length
		size_t   rem_len;
	};

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

	bool initialize(usb_driver_base* const driver, const uint8_t ep0size, const buffer_adapter& tx_buf, const buffer_adapter& rx_buf);
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

	// bool handle_ep0_rx(const USB_common::USB_EVENTS event, const uint8_t ep);
	// bool handle_ep0_tx(const USB_common::USB_EVENTS event, const uint8_t ep);
	bool handle_ep0_setup(const USB_common::USB_EVENTS event, const uint8_t ep);

	void handle_ep_rx(const USB_common::USB_EVENTS event, const uint8_t ep);
	void handle_ep_tx(const USB_common::USB_EVENTS event, const uint8_t ep);

	virtual USB_common::USB_RESP process_request(Control_request* const req);

	virtual USB_common::USB_RESP handle_device_request(Control_request* const req);
	virtual USB_common::USB_RESP handle_iface_request(Control_request* const req);
	virtual USB_common::USB_RESP handle_ep_request(Control_request* const req);

	virtual bool set_configuration(const uint8_t bConfigurationValue);
	virtual bool get_configuration(uint8_t* const bConfigurationValue);

	virtual void handle_ctrl_req_complete();

	void stall_control_ep(const uint8_t ep);

	buffer_adapter m_rx_buffer;
	buffer_adapter m_tx_buffer;

	enum class USB_CONTROL_STATE
	{
		//no control traffic
		IDLE,
		//waiting to rx control trafic
		RXDATA,
		//waiting to rx control trafic
		TXDATA,
		//waiting to send zlp
		TX_ZLP,
		//sent last data, waiting for completion
		LASTDATA,
		//dev to host
		STATUS_IN,
		//host to dev
		STATUS_OUT
	};
	USB_CONTROL_STATE m_control_state;

	usb_driver_base* m_driver;

	Control_request m_ctrl_req;
	std::array<uint8_t, 64> m_ctrl_req_data;
};