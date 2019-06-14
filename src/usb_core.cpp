#include "libusb_dev_cpp/usb_core.hpp"

#include "libusb_dev_cpp/descriptor/Device_descriptor.hpp"

#include "common_util/Byte_util.hpp"

#include "uart1_printf.hpp"

#include <algorithm>
#include <functional>

USB_core::USB_core()
{
	m_driver = nullptr;
}

USB_core::~USB_core()
{
	
}

bool USB_core::initialize(usb_driver_base* const driver, const uint8_t ep0size, const buffer_adapter& tx_buf, const buffer_adapter& rx_buf)
{
	m_address = 0;

	m_driver = driver;

	m_tx_buffer = tx_buf;
	m_tx_buffer.reset();
	m_rx_buffer = rx_buf;
	m_rx_buffer.reset();

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

void USB_core::set_address(const uint8_t addr)
{
	m_address = addr;
	m_driver->set_address(addr);	
}

bool USB_core::handle_reset()
{
	set_address(0);

	m_setup_complete_callback = nullptr;

	usb_driver_base::ep_cfg ep0;
	ep0.num  = 0;
	if(m_driver->get_speed() == USB_common::USB_SPEED::LS)
	{
		ep0.size = 8;
	}
	else
	{
		ep0.size = 64;
	}
	ep0.type = usb_driver_base::EP_TYPE::CONTROL;
	m_driver->ep_config(ep0);

	m_driver->set_ep_rx_callback(0x00, std::bind(&USB_core::handle_ep0_rx, this, std::placeholders::_1, std::placeholders::_2));
	m_driver->set_ep_tx_callback(0x00, std::bind(&USB_core::handle_ep0_tx, this, std::placeholders::_1, std::placeholders::_2));
	m_driver->set_ep_setup_callback(0x00, std::bind(&USB_core::handle_ep0_setup, this, std::placeholders::_1, std::placeholders::_2));

	return true;
}

bool USB_core::handle_event(const USB_common::USB_EVENTS evt, const uint8_t ep)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep);

	USB_common::Event_callback func = nullptr;
	bool ret = false;
	switch(evt)
	{
		case USB_common::USB_EVENTS::RESET:
		{
			ret = handle_reset();
			break;
		}
		case USB_common::USB_EVENTS::EP_RX:
		{
			func = m_driver->get_ep_rx_callback(ep_addr);
			if(func)
			{
				func(evt, ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::EP_TX:
		{
			func = m_driver->get_ep_tx_callback(ep_addr);
			if(func)
			{
				func(evt, ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::SETUP_PACKET_RX:
		{
			func = m_driver->get_ep_setup_callback(ep_addr);
			if(func)
			{
				func(evt, ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::SETUP_TRX_DONE:
		{
			return false;			
		}
		case USB_common::USB_EVENTS::EARLY_SUSPEND:
		{
			//we will suspend soon
			break;
		}
		case USB_common::USB_EVENTS::SUSPEND:
		{
			//we are suspended
			break;
		}
		default:
		{
			break;
		}
	}

	func = m_driver->get_event_callback(ep_addr);//todo use the get addr helper func
	if(func)
	{
		func(evt, ep);
		ret = true;
	}

	return ret;
}

bool USB_core::handle_ep0_setup(const USB_common::USB_EVENTS event, const uint8_t ep)
{
	if(event != USB_common::USB_EVENTS::SETUP_PACKET_RX)
	{
		return false;
	}

	//init
	m_control_state = USB_CONTROL_STATE::IDLE;
	m_setup_complete_callback = nullptr;

	//force read & process
	handle_ep0_rx(event, ep);

	return true;
}

void USB_core::handle_ep0_rx(const USB_common::USB_EVENTS event, const uint8_t ep)
{
	switch(m_control_state)
	{
		case USB_CONTROL_STATE::IDLE:
		{
			Setup_packet::Setup_packet_array setup_packet_array;
			if(0x08 != m_driver->ep_read(ep, setup_packet_array.data(), setup_packet_array.size()))
			{
				stall_control_ep(ep);
				return;
			}

			//decode
			m_ctrl_req = Control_request();
			if(!m_ctrl_req.setup_packet.deserialize(setup_packet_array))
			{
				return;
			}

			Request_type req_type;
			if(!m_ctrl_req.setup_packet.get_request_type(&req_type))
			{
				return;
			}

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.bmRequestType: 0x%02X",
			// 	m_ctrl_req.setup_packet.bmRequestType
			// 	);

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.bRequest: 0x%02X",
			// 	m_ctrl_req.setup_packet.bRequest
			// 	);

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wValue: 0x%04X",
			// 	m_ctrl_req.setup_packet.wValue
			// 	);

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wIndex: 0x%04X",
			// 	m_ctrl_req.setup_packet.wIndex
			// 	);

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wLength: 0x%04X",
			// 	m_ctrl_req.setup_packet.wLength
			// 	);

			//check if we need to read data from the host
			if((req_type.data_dir == Request_type::DATA_DIR::HOST_TO_DEV))
			{
				if(m_ctrl_req.setup_packet.wLength == 0)
				{
					//zero len incoming data segment
					//skip read state, process event
					break;
				}

				//setup read
				if(m_ctrl_req.setup_packet.wLength > m_rx_buffer.buf_maxsize)
				{
					stall_control_ep(ep);
					return;
				}

				//we have additional host->dev data, so advance the state machine to wait for the data
				m_control_state = USB_CONTROL_STATE::RXDATA;
				
				//clear this out, start a read of up to setup_packet.wLength for the data portion
				//this will happen during the event processing if needed
				m_rx_buffer.reset();
				m_rx_buffer.rem_len = m_ctrl_req.setup_packet.wLength;

				//don't process the event yet, we need to read more
				//there might be host->dev data coming
				//processing will continue on the next read event
				return;
			}
			else
			{
				///Nothing to do
			}

			//DEV_TO_HOST, we can process and respond instead
			break;
		}
		case USB_CONTROL_STATE::RXDATA:
		{
			const int rxlen = m_driver->ep_read(ep, m_rx_buffer.curr_ptr, m_rx_buffer.rem_len);
			if(rxlen < 0)
			{
				//error
				stall_control_ep(ep);
				return;				
			}
			else if(m_rx_buffer.rem_len < size_t(rxlen))
			{
				//we got too much data, that is weird
				stall_control_ep(ep);
				return;
			}
			else if(m_rx_buffer.rem_len != size_t(rxlen))
			{
				//keep reading
				m_rx_buffer.curr_ptr += rxlen;
				m_rx_buffer.rem_len -= rxlen;

				//skip evt processing
				return;
			}
			break;
		}
		case USB_CONTROL_STATE::STATUS_OUT:
		{
			//handle status out packet
			m_rx_buffer.reset();
			m_driver->ep_read(ep, m_rx_buffer.buf_ptr, m_rx_buffer.buf_maxsize);
			m_control_state = USB_CONTROL_STATE::IDLE;
			if(m_setup_complete_callback)
			{
				m_setup_complete_callback();
			}
			return;
		}
		default:
		{
			//unk error, reset control ep
			stall_control_ep(ep);
			break;
		}
	}

	Request_type req_type;
	if(!m_ctrl_req.setup_packet.get_request_type(&req_type))
	{
		//FAIL
		return;
	}

	switch(process_request(&m_ctrl_req))
	{
		case USB_common::USB_RESP::ACK:
		{
			//did the host ask us to send data? if so, send it
			if((req_type.data_dir == Request_type::DATA_DIR::DEV_TO_HOST))
			{
				if(m_tx_buffer.rem_len >= m_ctrl_req.setup_packet.wLength)
				{
					m_tx_buffer.rem_len = m_ctrl_req.setup_packet.wLength;
				}

				m_control_state = USB_CONTROL_STATE::TXDATA;
				handle_ep0_tx(event, ep | 0x80);
			}
			else
			{
				//otherwise send a zlp status packet
				m_tx_buffer.reset();
				m_driver->ep_write(ep | 0x80, 0, 0);
				m_control_state = USB_CONTROL_STATE::STATUS_IN;
			}
			break;
		}
		case USB_common::USB_RESP::NAK:
		{
			m_control_state = USB_CONTROL_STATE::STATUS_IN;
			break;
		}
		default:
		{
			//invalid state, reset control endpoint
			stall_control_ep(ep);
			break;
		}
	}
	
}
void USB_core::handle_ep0_tx(const USB_common::USB_EVENTS event, const uint8_t ep)
{
	switch(m_control_state)
	{
		case USB_CONTROL_STATE::TXDATA:
		{
			const size_t ep0size = m_driver->get_ep0_config().size;
			const size_t num_to_write = std::min(m_tx_buffer.rem_len, ep0size);

			int num_wrote = m_driver->ep_write(ep | 0x80, m_tx_buffer.curr_ptr, num_to_write);

			if(num_wrote < 0)
			{
				// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "ep_write error");
			}
			else
			{
				m_tx_buffer.curr_ptr += num_wrote;
				m_tx_buffer.rem_len  -= num_wrote;
				// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "wrote %d, left %d", num_wrote, m_tx_buffer.rem_len);
			}

			if(m_tx_buffer.rem_len == 0)
			{
				if(num_to_write != ep0size)
				{
					m_control_state = USB_CONTROL_STATE::TXCOMP;
				}
				else
				{
					m_control_state = USB_CONTROL_STATE::TXZLP;
				}
			}
			break;
		}
		case USB_CONTROL_STATE::TXZLP:
		{
			const int ret = m_driver->ep_write(ep | 0x80, nullptr, 0);
			if(ret != 0)
			{
				// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "TXZLP had error on ep_write");
			}

			m_control_state = USB_CONTROL_STATE::TXCOMP;
			break;
		}
		case USB_CONTROL_STATE::TXCOMP:
		{
			m_control_state = USB_CONTROL_STATE::STATUS_OUT;
			break;	
		}
		case USB_CONTROL_STATE::STATUS_IN:
		{
			m_control_state = USB_CONTROL_STATE::IDLE;
			//tx complete, so status in ack sent
			//call the deffered process callback
			if(m_setup_complete_callback)
			{
				m_setup_complete_callback();
			}
			return;
		}
		default:
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "default, event %d, state %d", event, m_control_state);
			break;
		}
	}
}

USB_common::USB_RESP USB_core::process_request(Control_request* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	Request_type request_type;
	if(!req->setup_packet.get_request_type(&request_type))
	{
		return r;
	}

	switch(request_type.type)
	{
		case Request_type::TYPE::STANDARD:
		{	
			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "STANDARD request");
			switch(request_type.recipient)
			{
				case Request_type::RECIPIENT::DEVICE:
				{
					r = handle_std_device_request(req);
					break;
				}
				case Request_type::RECIPIENT::INTERFACE:
				{
					r = handle_std_iface_request(req);
					break;
				}
				case Request_type::RECIPIENT::ENDPOINT:
				{
					r = handle_std_ep_request(req);
					break;
				}
				default:
				{
					r = USB_common::USB_RESP::FAIL;
					break;
				}
			}
			break;
		}
		case Request_type::TYPE::CLASS:
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "CLASS request");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		case Request_type::TYPE::VENDOR:
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "VENDOR request");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		case Request_type::TYPE::RESERVED:
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "RESERVED request");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		default:
		{
			r = USB_common::USB_RESP::FAIL;
		}
	}

	return r;
}

USB_common::USB_RESP USB_core::handle_std_device_request(Control_request* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::DEVICE_REQUEST>(req->setup_packet.bRequest))
	{
		case Setup_packet::DEVICE_REQUEST::GET_STATUS:
		{
			m_tx_buffer.reset();
			m_tx_buffer.buf_ptr[0] = 0;
			m_tx_buffer.buf_ptr[1] = 0;
			m_tx_buffer.rem_len = 2;

			// if(selfpowered)
			// {
			// 	m_tx_buffer.buf_ptr[0] |= (1U << 0);
			// }

			// if(remote_wakeup)
			// {
			// 	m_tx_buffer.buf_ptr[0] |= (1U << 1);
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
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_ADDRESS");

			if((req->setup_packet.wIndex != 0) || (req->setup_packet.wLength != 0))
			{
				// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_ADDRESS packet invalid");
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			if(req->setup_packet.wValue > 127)
			{
				// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_ADDRESS address invalid");
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "Queue SET_ADDRESS to %d", req->setup_packet.wValue);
			
			// m_address = req->setup_packet.wValue;
			m_setup_complete_callback = std::bind(&USB_core::set_address, this, req->setup_packet.wValue);
			// set_address(req->setup_packet.wValue);
			r = USB_common::USB_RESP::ACK;
			break;
		}
		// handled by child class
		case Setup_packet::DEVICE_REQUEST::GET_DESCRIPTOR:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "GET_DESCRIPTOR");

			Device_descriptor dev_desc;
			dev_desc.bcdUSB = Device_descriptor::build_bcd(2, 0, 0);
			dev_desc.bDeviceClass    = 0;
			dev_desc.bDeviceSubClass = 0;
			dev_desc.bDeviceProtocol = 0;
			dev_desc.bMaxPacketSize0 = m_driver->get_ep0_config().size;
			dev_desc.idVendor  = 0x0123;
			dev_desc.idProduct = 0x4567;
			dev_desc.bcdDevice = Device_descriptor::build_bcd(1, 0, 0);
			dev_desc.iManufacturer      = 0;
			dev_desc.iProduct           = 0;
			dev_desc.iSerialNumber      = 0;
			dev_desc.bNumConfigurations = 1;

			Device_descriptor::Device_descriptor_array desc_arr;
			if(!dev_desc.serialize(&desc_arr))
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			m_tx_buffer.reset();
			std::copy_n(desc_arr.data(), desc_arr.size(), m_tx_buffer.buf_ptr);
			m_tx_buffer.rem_len = desc_arr.size();

			//truncate if needed
			if(req->setup_packet.wLength < m_tx_buffer.rem_len)
			{
				m_tx_buffer.rem_len = req->setup_packet.wLength;
			}

			r = USB_common::USB_RESP::ACK;

			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_DESCRIPTOR:
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_DESCRIPTOR");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		case Setup_packet::DEVICE_REQUEST::GET_CONFIGURATION:
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "GET_CONFIGURATION");

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
			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_CONFIGURATION");
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
USB_common::USB_RESP USB_core::handle_std_iface_request(Control_request* const req)
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
USB_common::USB_RESP USB_core::handle_std_ep_request(Control_request* const req)
{
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

void USB_core::stall_control_ep(const uint8_t ep)
{
	m_driver->ep_stall(ep & 0x7F);
	m_driver->ep_stall(ep | 0x80);

	m_control_state = USB_CONTROL_STATE::IDLE;
}