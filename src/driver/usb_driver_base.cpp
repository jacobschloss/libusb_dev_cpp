#include "libusb_dev_cpp/driver/usb_driver_base.hpp"

usb_driver_base::usb_driver_base()
{
	m_control_callback = nullptr;
	m_control_transfer_complete_callback = nullptr;
	m_set_configuration_callback = nullptr;
	m_get_descriptor_callback = nullptr;

	m_event_callbacks.fill(nullptr);
	m_ep_rx_callbacks.fill(nullptr);
	m_ep_tx_callbacks.fill(nullptr);
	m_ep_setup_callbacks.fill(nullptr);
}

bool usb_driver_base::set_ep_rx_callback(const uint8_t ep, const USB_common::Event_callback& func)
{
	m_ep_rx_callbacks[ep] = func;

	return true;
}

bool usb_driver_base::set_ep_tx_callback(const uint8_t ep, const USB_common::Event_callback& func)
{
	m_ep_tx_callbacks[ep & 0x7F] = func;

	return true;
}

bool usb_driver_base::set_ep_setup_callback(const uint8_t ep, const USB_common::Event_callback& func)
{
	m_ep_setup_callbacks[ep] = func;

	return true;
}