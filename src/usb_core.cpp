#include "libusb_dev_cpp/usb_core.hpp"

#include "uart1_printf.hpp"

#include <functional>

USB_core::USB_core()
{
	m_driver = nullptr;
}

bool USB_core::initialize(usb_driver_base* const driver, const uint8_t ep0size)
{
	m_driver = driver;

	return true;
}

bool USB_core::poll()
{
	m_driver->poll(std::bind(&USB_core::handle_event, this, std::placeholders::_1, std::placeholders::_2));
	return true;
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

void USB_core::set_control_callback(const USB_common::Control_callback& func)
{

}
void USB_core::set_config_callback(const USB_common::Set_configuration_callback& func)
{

}
void USB_core::set_descriptor_callback(const USB_common::Get_descriptor_callback& func)
{

}

bool USB_core::handle_reset()
{
	m_driver->ep_config(0);

	m_driver->get_status().active_device_cfg = 0;

	m_driver->set_ep_rx_callback(0x00, std::bind(&USB_core::handle_ep0_rx, this, std::placeholders::_1, std::placeholders::_2));
	m_driver->set_ep_tx_callback(0x80, std::bind(&USB_core::handle_ep0_tx, this, std::placeholders::_1, std::placeholders::_2));
	m_driver->set_ep_setup_callback(0x00, std::bind(&USB_core::handle_ep0_setup, this, std::placeholders::_1, std::placeholders::_2));

	m_driver->set_address(0);	

	return true;
}

bool USB_core::handle_event(const USB_common::USB_EVENTS evt, const uint8_t ep)
{
	USB_common::Event_callback func = nullptr;
	bool ret = false;
	switch(evt)
	{
		case USB_common::USB_EVENTS::RESET:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_event USB_EVENTS::RESET");

			ret = handle_reset();
			break;
		}
		case USB_common::USB_EVENTS::EPRX:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_event USB_EVENTS::EPRX");

			func = m_driver->get_ep_rx_callback(ep);
			break;
		}
		case USB_common::USB_EVENTS::EPTX:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_event USB_EVENTS::EPTX");

			func = m_driver->get_ep_tx_callback(ep);
			break;
		}
		case USB_common::USB_EVENTS::EPSETUP:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_event USB_EVENTS::EPSETUP");

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
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_ep0_rx");

	return false;
}

bool USB_core::handle_ep0_tx(const USB_common::USB_EVENTS event, const uint8_t ep)
{
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_ep0_tx");

	return false;
}

bool USB_core::handle_ep0_setup(const USB_common::USB_EVENTS event, const uint8_t ep)
{
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_ep0_setup");

	if(event != USB_common::USB_EVENTS::EPSETUP)
	{
		return false;
	}

	//init
	m_driver->get_status().control_state = usb_driver_base::USB_STATE::IDLE;

	//force read
	process_eprx(ep);

	return true;
}

void USB_core::process_eprx(const uint8_t ep)
{
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "process_eprx");

	switch(m_driver->get_status().control_state)
	{
		case usb_driver_base::USB_STATE::IDLE:
		{
			Setup_packet::Setup_packet_array temp_buffer;

			if(0x08 != m_driver->ep_read(ep, temp_buffer.data(), temp_buffer.size()))
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "process_eprx ep_read fail");
			}

			Control_request req;
			if(!req.setup_packet.deserialize(temp_buffer))
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.deserialize fail");
			}

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet: 0x%02X%02X%02X%02X%02X%02X%02X%02X", 
					temp_buffer[0],
					temp_buffer[1],
					temp_buffer[2],
					temp_buffer[3],
					temp_buffer[4],
					temp_buffer[5],
					temp_buffer[6],
					temp_buffer[7]
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.bmRequestType: 0x%02X",
				req.setup_packet.bmRequestType
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.bRequest: 0x%02X",
				req.setup_packet.bRequest
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wValue: 0x%04X",
				req.setup_packet.wValue
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wIndex: 0x%04X",
				req.setup_packet.wIndex
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wLength: 0x%04X",
				req.setup_packet.wLength
				);

			break;
		}
		case usb_driver_base::USB_STATE::RXDATA:
		{
			break;
		}
		case usb_driver_base::USB_STATE::STATUS_OUT:
		{
			break;
		}
		default:
		{
			break;
		}
	}
}
void USB_core::process_eptx(const uint8_t ep)
{

}