/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

bool usb_driver_base::ep_cfg::is_valid() const
{
	bool valid = true;
	switch(type)
	{
		case EP_TYPE::CONTROL:
		case EP_TYPE::ISOCHRONUS:
		case EP_TYPE::INTERRUPT:
		case EP_TYPE::BULK:
		{
			valid = true;
			break;
		}
		case EP_TYPE::UNCONF:
		default:
		{
			valid = false;
			break;
		}
	}

	return valid;
}

usb_driver_base::usb_driver_base()
{
	m_event_callbacks.fill(nullptr);
	m_ep_rx_callbacks.fill(nullptr);
	m_ep_tx_callbacks.fill(nullptr);
	m_ep_setup_callbacks.fill(nullptr);
}

bool usb_driver_base::set_ep_rx_callback(const uint8_t ep_addr, const USB_common::Event_callback& func)
{
	m_ep_rx_callbacks[ep_addr] = func;

	return true;
}

bool usb_driver_base::set_ep_tx_callback(const uint8_t ep_addr, const USB_common::Event_callback& func)
{
	m_ep_tx_callbacks[ep_addr] = func;

	return true;
}

bool usb_driver_base::set_ep_setup_callback(const uint8_t ep_addr, const USB_common::Event_callback& func)
{
	m_ep_setup_callbacks[ep_addr] = func;

	return true;
}

bool usb_driver_base::handle_reset()
{
	return true;
}

bool usb_driver_base::handle_enum_done()
{
	return true;
}
