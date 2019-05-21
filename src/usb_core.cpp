#include "libusb_dev_cpp/usb_core.hpp"

#include <functional>

bool USB_core::initialize(usb_driver_base* driver, const uint8_t ep0size)
{
	return false;
}

bool USB_core::poll()
{
	return false;
}

bool USB_core::enable()
{
	return m_driver->enable();	
}
bool USB_core::disable()
{
	return m_driver->disable();	
}

bool USB_core::connect()
{
	return m_driver->connect();
}
bool USB_core::disconnect()
{
	return m_driver->disconnect();
}

bool USB_core::handle_event(const USB_common::USB_EVENTS evt, const uint8_t ep)
{
	bool ret = false;
	switch(evt)
	{
		case USB_common::USB_EVENTS::RESET:
		{
			m_driver->ep_config(0);

			m_driver->get_status().active_device_cfg = 0;

			m_driver->set_ep_rx_callback(0x00, std::bind(&USB_core::handle_ep0_rx, this, std::placeholders::_1, std::placeholders::_2));
			m_driver->set_ep_tx_callback(0x80, std::bind(&USB_core::handle_ep0_tx, this, std::placeholders::_1, std::placeholders::_2));
			m_driver->set_ep_setup_callback(0x00, std::bind(&USB_core::handle_ep0_setup, this, std::placeholders::_1, std::placeholders::_2));

			m_driver->set_address(0);
			break;
		}
		case USB_common::USB_EVENTS::EPRX:
		{
			const auto& func = m_device->get_ep_rx_callback();
			if(func)
			{
				func(evt, ep);
			}

			break;
		}
		case USB_common::USB_EVENTS::EPTX:
		{
			const auto& func = m_device->get_ep_tx_callback();
			if(func)
			{
				func(evt, ep);
			}

			break;
		}
		case USB_common::USB_EVENTS::EPSETUP:
		{
			const auto& func = m_device->get_ep_setup_callback();
			if(func)
			{
				func(evt, ep);
			}

			break;
		}
		default:
		{
			const auto& func = m_device->get_event_callback(ep);
			if(func)
			{
				func(evt, ep);
			}

			break;
		}
	}

	return ret;
}

bool USB_core::handle_ep0_rx(const USB_common::USB_EVENTS event, const uint8_t ep)
{
	
}

bool USB_core::handle_ep0_tx(const USB_common::USB_EVENTS event, const uint8_t ep)
{

}

bool USB_core::handle_ep0_setup(const USB_common::USB_EVENTS event, const uint8_t ep)
{

}
