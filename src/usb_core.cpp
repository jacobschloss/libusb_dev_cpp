#include "libusb_dev_cpp/usb_core.hpp"

#include <functional>

USB_core::USB_core()
{
	m_driver = nullptr;
}

bool USB_core::initialize(usb_driver_base* const driver, const uint8_t ep0size)
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

bool USB_core::handle_reset()
{
	m_driver->ep_config(0);

	m_driver->get_status().active_device_cfg = 0;

	m_driver->set_ep_rx_callback(0x00, std::bind(&USB_core::handle_ep0_rx, this, std::placeholders::_1, std::placeholders::_2));
	m_driver->set_ep_tx_callback(0x80, std::bind(&USB_core::handle_ep0_tx, this, std::placeholders::_1, std::placeholders::_2));
	m_driver->set_ep_setup_callback(0x00, std::bind(&USB_core::handle_ep0_setup, this, std::placeholders::_1, std::placeholders::_2));

	m_driver->set_address(0);	
}

bool USB_core::handle_event(const USB_common::USB_EVENTS evt, const uint8_t ep)
{
	USB_common::Event_callback func = nullptr;
	bool ret = false;
	switch(evt)
	{
		case USB_common::USB_EVENTS::RESET:
		{
			ret = handle_reset();
			break;
		}
		case USB_common::USB_EVENTS::EPRX:
		{
			func = m_driver->get_ep_rx_callback(ep);
			break;
		}
		case USB_common::USB_EVENTS::EPTX:
		{
			func = m_driver->get_ep_tx_callback(ep);
			break;
		}
		case USB_common::USB_EVENTS::EPSETUP:
		{
			func = m_driver->get_ep_setup_callback(ep);
			break;
		}
		default:
		{
			func = m_driver->get_event_callback(ep);
			break;
		}
	}

	if(func)
	{
		func(evt, ep);
		ret = true;
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
