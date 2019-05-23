#include "libusb_dev_cpp/usb_core.hpp"

#include "uart1_printf.hpp"

#include <functional>

USB_core::USB_core()
{
	m_driver = nullptr;
}

USB_core::~USB_core()
{
	
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
			if(func)
			{
				func(evt, ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::EPTX:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_event USB_EVENTS::EPTX");

			func = m_driver->get_ep_tx_callback(ep);
			if(func)
			{
				func(evt, ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::EPSETUP:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_event USB_EVENTS::EPSETUP");

			func = m_driver->get_ep_setup_callback(ep);
			if(func)
			{
				func(evt, ep);
			}
			break;
		}
		default:
		{
			break;
		}
	}

	func = m_driver->get_event_callback(ep);
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
	handle_ep_rx(ep);

	return true;
}

void USB_core::handle_ep_rx(const uint8_t ep)
{
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_ep_rx");

	switch(m_driver->get_status().control_state)
	{
		case usb_driver_base::USB_STATE::IDLE:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_rx", "USB_STATE::IDLE");

			Setup_packet::Setup_packet_array temp_buffer;

			if(0x08 != m_driver->ep_read(ep, temp_buffer.data(), temp_buffer.size()))
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "handle_ep_rx ep_read fail");
			}

			m_ctrl_req = Control_request(m_ctrl_req_data.data(), m_ctrl_req_data.size());
			if(!m_ctrl_req.setup_packet.deserialize(temp_buffer))
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.deserialize fail");
			}

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.bmRequestType: 0x%02X",
				m_ctrl_req.setup_packet.bmRequestType
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.bRequest: 0x%02X",
				m_ctrl_req.setup_packet.bRequest
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wValue: 0x%04X",
				m_ctrl_req.setup_packet.wValue
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wIndex: 0x%04X",
				m_ctrl_req.setup_packet.wIndex
				);

			uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wLength: 0x%04X",
				m_ctrl_req.setup_packet.wLength
				);

			m_driver->get_status().control_state = usb_driver_base::USB_STATE::RXDATA;
			
			m_rx_buffer.data_buf = m_ctrl_req_data.data();
			m_rx_buffer.data_ptr = m_ctrl_req_data.data();
			m_rx_buffer.data_count = m_ctrl_req.setup_packet.wLength;
			m_rx_buffer.data_maxsize = m_ctrl_req_data.size();

			break;
		}
		case usb_driver_base::USB_STATE::RXDATA:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_rx", "USB_STATE::RXDATA");

			const size_t rxlen = m_driver->ep_read(ep, m_rx_buffer.data_ptr, m_rx_buffer.data_count);
			if(m_rx_buffer.data_count < rxlen)
			{

			}
			else if(m_rx_buffer.data_count != rxlen)
			{
				m_rx_buffer.data_count -= rxlen;
				m_rx_buffer.data_ptr += rxlen;
			}

			break;
		}
		case usb_driver_base::USB_STATE::STATUS_OUT:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_rx", "USB_STATE::STATUS_OUT");

			m_driver->ep_read(ep, m_rx_buffer.data_ptr, m_rx_buffer.data_maxsize);

			m_driver->get_status().control_state = usb_driver_base::USB_STATE::IDLE;

			handle_ctrl_req_complete();
			break;
		}
		default:
		{
			break;
		}

		switch(process_request(&m_ctrl_req))
		{
			case USB_common::USB_RESP::ACK:
			{
				if(m_ctrl_req.setup_packet.bmRequestType & 0x80)
				{
					if(m_rx_buffer.data_count >= m_ctrl_req.setup_packet.wLength)
					{
						m_rx_buffer.data_count = m_ctrl_req.setup_packet.wLength;
						m_driver->get_status().control_state = usb_driver_base::USB_STATE::TXDATA;
					}
					else
					{
						m_driver->get_status().control_state = usb_driver_base::USB_STATE::TX_ZLP;
					}

					handle_ep_tx(ep | 0x80);
				}
				else
				{
					m_driver->ep_write(ep | 0x80, 0, 0);
					m_driver->get_status().control_state = usb_driver_base::USB_STATE::STATUS_IN;
				}
				break;
			}
			case USB_common::USB_RESP::NAK:
			{
				m_driver->get_status().control_state = usb_driver_base::USB_STATE::STATUS_IN;
				break;
			}
			default:
			{
				//TODO stall
				break;
			}
		}
	}
}
void USB_core::handle_ep_tx(const uint8_t ep)
{

}

USB_common::USB_RESP USB_core::process_request(Control_request* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	Request_type request_type;
	req->setup_packet.get_request_type(&request_type);

	if(request_type.type == Request_type::TYPE::STANDARD)
	{
		switch(request_type.recipient)
		{
			case Request_type::RECIPIENT::DEVICE:
			{
				r = handle_device_request(req);
			}
			case Request_type::RECIPIENT::INTERFACE:
			{
				r = handle_iface_request(req);
			}
			case Request_type::RECIPIENT::ENDPOINT:
			{
				r = handle_ep_request(req);
			}
			default:
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}
		}
	}

	return USB_common::USB_RESP::FAIL;
}

USB_common::USB_RESP USB_core::handle_device_request(Control_request* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::DEVICE_REQUEST>(req->setup_packet.bRequest))
	{
		case Setup_packet::DEVICE_REQUEST::GET_STATUS:
		{
			break;
		}
		case Setup_packet::DEVICE_REQUEST::CLEAR_FEATURE:
		{
			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_FEATURE:
		{
			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_ADDRESS:
		{
			break;
		}
		case Setup_packet::DEVICE_REQUEST::GET_DESCRIPTOR:
		{
			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_DESCRIPTOR:
		{
			break;
		}
		case Setup_packet::DEVICE_REQUEST::GET_CONFIGURATION:
		{
			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_CONFIGURATION:
		{
			break;
		}
		default:
		{
			r = USB_common::USB_RESP::FAIL;
			break;
		}
	}
	return r;
}
USB_common::USB_RESP USB_core::handle_iface_request(Control_request* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::INTERFACE_REQUEST>(req->setup_packet.bRequest))
	{
		case Setup_packet::INTERFACE_REQUEST::GET_STATUS:
		{
			req->data_stage_buffer[0] = 0;
			req->data_stage_buffer[1] = 0;
			r = USB_common::USB_RESP::ACK;
			break;
		}
		default:
		{
			r = USB_common::USB_RESP::FAIL;
			break;
		}
	}
	return r;
}
USB_common::USB_RESP USB_core::handle_ep_request(Control_request* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::ENDPOINT_REQUEST>(req->setup_packet.bRequest))
	{
		case Setup_packet::ENDPOINT_REQUEST::GET_STATUS:
		{
			m_driver->ep_stall(req->setup_packet.wIndex);
			r = USB_common::USB_RESP::ACK;
		}
		case Setup_packet::ENDPOINT_REQUEST::CLEAR_FEATURE:
		{
			m_driver->ep_unstall(req->setup_packet.wIndex);
			r = USB_common::USB_RESP::ACK;
		}
		case Setup_packet::ENDPOINT_REQUEST::SET_FEATURE:
		{
			req->data_stage_buffer[0] = m_driver->ep_is_stalled(req->setup_packet.wIndex) ? 1 : 0;
			req->data_stage_buffer[1] = 0;
			r = USB_common::USB_RESP::ACK;
		}
		default:
		{
			r = USB_common::USB_RESP::FAIL;
			break;
		}
	}
	return r;
}

void USB_core::handle_ctrl_req_complete()
{
	
}