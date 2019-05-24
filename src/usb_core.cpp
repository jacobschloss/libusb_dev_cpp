#include "libusb_dev_cpp/usb_core.hpp"

#include "common_util/Byte_util.hpp"

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
	usb_driver_base::ep_cfg ep0;
	ep0.num  = 0;
	ep0.size = 8;
	ep0.type = usb_driver_base::EP_TYPE::CONTROL;
	m_driver->ep_config(ep0);

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
void USB_core::handle_ep_tx(const uint8_t ep)
{
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "USB_STATE::STATUS_OUT");

	switch(m_driver->get_status().control_state)
	{
		case usb_driver_base::USB_STATE::TXDATA:
		case usb_driver_base::USB_STATE::TX_ZLP:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::TXDATA", "USB_STATE::STATUS_OUT");

			const size_t ep0size = 8;
			const size_t num_to_write = std::min(m_rx_buffer.data_count, ep0size);

			m_driver->ep_write(ep | 0x80, m_rx_buffer.data_ptr, num_to_write);

			m_rx_buffer.data_ptr   += num_to_write;
			m_rx_buffer.data_count -= num_to_write;

			if(m_rx_buffer.data_count == 0)
			{
				if((m_driver->get_status().control_state == usb_driver_base::USB_STATE::TXDATA) || (num_to_write != ep0size))
				{
					m_driver->get_status().control_state = usb_driver_base::USB_STATE::LASTDATA;
				}
			}

			break;
		}
		case usb_driver_base::USB_STATE::LASTDATA:
		{
			m_driver->get_status().control_state = usb_driver_base::USB_STATE::STATUS_OUT;
			break;
		}
		case usb_driver_base::USB_STATE::STATUS_IN:
		{
			m_driver->get_status().control_state = usb_driver_base::USB_STATE::IDLE;
			break;
		}
		default:
		{
			break;
		}
	}
}

USB_common::USB_RESP USB_core::process_request(Control_request* const req)
{
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "");

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
				break;
			}
			case Request_type::RECIPIENT::INTERFACE:
			{
				r = handle_iface_request(req);
				break;
			}
			case Request_type::RECIPIENT::ENDPOINT:
			{
				r = handle_ep_request(req);
				break;
			}
			default:
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}
		}
	}

	return r;
}

USB_common::USB_RESP USB_core::handle_device_request(Control_request* const req)
{
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_device_request", "");

	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::DEVICE_REQUEST>(req->setup_packet.bRequest))
	{
		case Setup_packet::DEVICE_REQUEST::GET_STATUS:
		{
			req->data_stage_buffer[0] = 0;
			req->data_stage_buffer[1] = 0;

			// if(selfpowered)
			// {
			// 	req->data_stage_buffer[0] |= (1U << 0);
			// }

			// if(remote_wakeup)
			// {
			// 	req->data_stage_buffer[0] |= (1U << 1);
			// }

			r = USB_common::USB_RESP::ACK;
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
			if(!m_driver->set_address(req->setup_packet.wValue))
			{
				r = USB_common::USB_RESP::FAIL;
			}
			else
			{
				r = USB_common::USB_RESP::ACK;
			}
			break;
		}
		// handled by child class
		case Setup_packet::DEVICE_REQUEST::GET_DESCRIPTOR:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_device_request", "GET_DESCRIPTOR");
			
			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_DESCRIPTOR:
		{
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		case Setup_packet::DEVICE_REQUEST::GET_CONFIGURATION:
		{
			if(
				(req->setup_packet.wValue  != 0) ||
				(req->setup_packet.wIndex  != 0) ||
				(req->setup_packet.wLength != 1)
				)
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			if(!get_configuration(&(req->data_stage_buffer[0])))
			{
				r = USB_common::USB_RESP::FAIL;		
			}
			else
			{
				r = USB_common::USB_RESP::ACK;
			}

			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_CONFIGURATION:
		{
			if(
				(Byte_util::get_b1(req->setup_packet.wValue) != 0) ||
				(req->setup_packet.wIndex  != 0)                   ||
				(req->setup_packet.wLength != 0)
				)
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			const uint8_t bConfigurationValue = Byte_util::get_b0(req->setup_packet.wValue);
			if(!set_configuration(bConfigurationValue))
			{
				r = USB_common::USB_RESP::FAIL;		
			}
			else
			{
				r = USB_common::USB_RESP::ACK;
			}
			
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
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_iface_request", "");

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
	uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_request", "");

	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	const Setup_packet::FEATURE_SELECTOR feature = static_cast<Setup_packet::FEATURE_SELECTOR>(req->setup_packet.wValue);
	if(feature != Setup_packet::FEATURE_SELECTOR::ENDPOINT_HALT)
	{
		return r;
	}

	const uint8_t endpoint_idx = Byte_util::get_b0(req->setup_packet.wIndex);

	switch(static_cast<Setup_packet::ENDPOINT_REQUEST>(req->setup_packet.bRequest))
	{
		case Setup_packet::ENDPOINT_REQUEST::SET_FEATURE:
		{
			m_driver->ep_stall(endpoint_idx);
			r = USB_common::USB_RESP::ACK;
		}
		case Setup_packet::ENDPOINT_REQUEST::CLEAR_FEATURE:
		{
			m_driver->ep_unstall(endpoint_idx);
			r = USB_common::USB_RESP::ACK;
		}
		case Setup_packet::ENDPOINT_REQUEST::GET_STATUS:
		{
			if(m_driver->ep_is_stalled(endpoint_idx))
			{
				req->data_stage_buffer[0] = (1U << 0);
			}
			else
			{
				req->data_stage_buffer[0] = 0;
			}
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

bool USB_core::set_configuration(const uint8_t bConfigurationValue)
{
	if(bConfigurationValue != 0)
	{
		return false;
	}

	return true;
}
bool USB_core::get_configuration(uint8_t* const bConfigurationValue)
{
	*bConfigurationValue = 0;
	return true;
}

void USB_core::handle_ctrl_req_complete()
{
	
}