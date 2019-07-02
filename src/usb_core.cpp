/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/usb_core.hpp"

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/descriptor/Device_descriptor.hpp"
#include "libusb_dev_cpp/descriptor/Configuration_descriptor.hpp"

#include "common_util/Byte_util.hpp"

#include "uart1_printf.hpp"

#include <algorithm>
#include <functional>

USB_core::USB_core()
{
	m_driver = nullptr;
	m_desc_table = nullptr;
}

USB_core::~USB_core()
{
	
}

bool USB_core::initialize(usb_driver_base* const driver, const uint8_t ep0size, const Buffer_adapter& tx_buf, const Buffer_adapter& rx_buf)
{
	m_address = 0;
	m_configuration = 0;

	m_driver = driver;

	m_tx_buffer = tx_buf;
	m_tx_buffer.reset();
	m_rx_buffer = rx_buf;
	m_rx_buffer.reset();

	return true;
}

void USB_core::set_descriptor_table(Descriptor_table* const desc_table)
{
	m_desc_table = desc_table;
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
	ep0.num = 0;
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
		case USB_common::USB_EVENTS::CTRL_SETUP_PHASE_DONE:
		{
			func = m_driver->get_ep_setup_callback(ep_addr);
			if(func)
			{
				func(evt, ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::CTRL_STATUS_PHASE:
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
		case USB_common::USB_EVENTS::NONE:
		{
			//ISR triggered but we don't care
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
	if(event != USB_common::USB_EVENTS::CTRL_SETUP_PHASE_DONE)
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
			const Setup_packet::Setup_packet_array* setup_packet_array = m_driver->get_last_setup_packet();

			//decode
			m_setup_packet = Setup_packet();
			if(!m_setup_packet.deserialize(*setup_packet_array))
			{
				return;
			}

			Request_type req_type;
			if(!m_setup_packet.get_request_type(&req_type))
			{
				return;
			}

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.bmRequestType: 0x%02X",
			// 	m_setup_packet.bmRequestType
			// 	);

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.bRequest: 0x%02X",
			// 	m_setup_packet.bRequest
			// 	);

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wValue: 0x%04X",
			// 	m_setup_packet.wValue
			// 	);

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wIndex: 0x%04X",
			// 	m_setup_packet.wIndex
			// 	);

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core", "setup_packet.wLength: 0x%04X",
			// 	m_setup_packet.wLength
			// 	);

			//check if we need to read data from the host
			if((req_type.data_dir == Request_type::DATA_DIR::HOST_TO_DEV))
			{
				if(m_setup_packet.wLength == 0)
				{
					//zero len incoming data segment
					//skip read state, process event
					break;
				}

				//setup read
				if(m_setup_packet.wLength > m_rx_buffer.max_size())
				{
					stall_control_ep(ep);
					return;
				}

				//we have additional host->dev data, so advance the state machine to wait for the data
				m_control_state = USB_CONTROL_STATE::RXDATA;
				
				//clear this out, start a read of up to setup_packet.wLength for the data portion
				//this will happen during the event processing if needed
				m_rx_buffer.reset();
				m_rx_buffer.rem_len = m_setup_packet.wLength;

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
			const Buffer_adapter_base* packet = m_driver->get_last_data_packet();

			size_t to_copy = std::min(packet->size(), m_rx_buffer.rem_len);
			std::copy_n(packet->data(), to_copy, m_rx_buffer.curr_ptr);

			if(m_rx_buffer.rem_len < packet->size())
			{
				//we got too much data, that is weird
				stall_control_ep(ep);
				return;
			}
			else if(m_rx_buffer.rem_len != packet->size())
			{
				//keep reading
				m_rx_buffer.curr_ptr += packet->size();
				m_rx_buffer.rem_len -= packet->size();

				//skip evt processing
				return;
			}
			break;
		}
		case USB_CONTROL_STATE::STATUS_OUT:
		{
			//handle status out packet
			m_rx_buffer.reset();
			
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
	if(!m_setup_packet.get_request_type(&req_type))
	{
		//FAIL
		return;
	}

	switch(process_request(&m_setup_packet))
	{
		case USB_common::USB_RESP::ACK:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "handle_ep0_rx", "process_request - ACK");

			//did the host ask us to send data? if so, send it
			if((req_type.data_dir == Request_type::DATA_DIR::DEV_TO_HOST))
			{
				if(m_tx_buffer.rem_len >= m_setup_packet.wLength)
				{
					m_tx_buffer.rem_len = m_setup_packet.wLength;
				}

				m_control_state = USB_CONTROL_STATE::TXDATA;
				handle_ep0_tx(event, ep | 0x80);
			}
			else
			{
				uart1_log<64>(LOG_LEVEL::INFO, "handle_ep0_rx", "process_request - zlp");

				//otherwise send a zlp status packet
				m_tx_buffer.reset();
				m_driver->ep_write(ep | 0x80, 0, 0);
				m_control_state = USB_CONTROL_STATE::STATUS_IN;
			}
			break;
		}
		case USB_common::USB_RESP::NAK:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "handle_ep0_rx", "process_request - NAK");

			m_control_state = USB_CONTROL_STATE::STATUS_IN;
			break;
		}
		default:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "handle_ep0_rx", "process_request - default");
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
			uart1_log<64>(LOG_LEVEL::INFO, "USB_CONTROL_STATE::TXDATA", "");

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
			uart1_log<64>(LOG_LEVEL::INFO, "USB_CONTROL_STATE::TXZLP", "");

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
			uart1_log<64>(LOG_LEVEL::INFO, "USB_CONTROL_STATE::TXCOMP", "");

			m_control_state = USB_CONTROL_STATE::STATUS_OUT;
			break;	
		}
		case USB_CONTROL_STATE::STATUS_IN:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_CONTROL_STATE::STATUS_IN", "");

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

USB_common::USB_RESP USB_core::process_request(Setup_packet* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	Request_type request_type;
	if(!req->get_request_type(&request_type))
	{
		return r;
	}

	switch(request_type.type)
	{
		case Request_type::TYPE::STANDARD:
		{	
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "STANDARD request");
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
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "CLASS request");
			// r = USB_common::USB_RESP::FAIL;
			r = USB_common::USB_RESP::ACK;
			break;
		}
		case Request_type::TYPE::VENDOR:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "VENDOR request");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		case Request_type::TYPE::RESERVED:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::process_request", "RESERVED request");
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

USB_common::USB_RESP USB_core::handle_std_device_request(Setup_packet* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::DEVICE_REQUEST>(req->bRequest))
	{
		case Setup_packet::DEVICE_REQUEST::GET_STATUS:
		{
			m_tx_buffer.reset();
			m_tx_buffer.insert(0);
			m_tx_buffer.insert(0);

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

			if((req->wIndex != 0) || (req->wLength != 0))
			{
				// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_ADDRESS packet invalid");
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			if(req->wValue > 127)
			{
				// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_ADDRESS address invalid");
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			// uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "Queue SET_ADDRESS to %d", req->wValue);
			
			// m_address = req->wValue;
			// m_setup_complete_callback = std::bind(&USB_core::set_address, this, req->wValue);
			set_address(req->wValue);
			r = USB_common::USB_RESP::ACK;
			break;
		}
		// handled by child class
		case Setup_packet::DEVICE_REQUEST::GET_DESCRIPTOR:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "GET_DESCRIPTOR");

			const USB_common::DESCRIPTOR_TYPE desc_type = static_cast<USB_common::DESCRIPTOR_TYPE>(Byte_util::get_b1(req->wValue));
			const uint8_t desc_index = Byte_util::get_b0(req->wValue);

			switch(desc_type)
			{
				case USB_common::DESCRIPTOR_TYPE::DEVICE:
				{
					Descriptor_table::Device_desc_const_ptr dev_desc = m_desc_table->get_device_descriptor(desc_index);
					if(!dev_desc)
					{
						r = USB_common::USB_RESP::FAIL;
						break;
					}

					Device_descriptor::Device_descriptor_array desc_arr;
					if(!dev_desc->serialize(&desc_arr))
					{
						r = USB_common::USB_RESP::FAIL;
						break;
					}

					m_tx_buffer.reset();

					//truncate if needed
					m_tx_buffer.insert(desc_arr.data(), std::min<size_t>(req->wLength, desc_arr.size()));

					r = USB_common::USB_RESP::ACK;
					break;
				}
				case USB_common::DESCRIPTOR_TYPE::CONFIGURATION:
				{
					Config_desc_table::Config_desc_const_ptr config_desc = m_desc_table->get_config_descriptor(desc_index);
					if(!config_desc)
					{
						r = USB_common::USB_RESP::FAIL;
						break;
					}

					m_tx_buffer.reset();

					if(!config_desc->serialize(&m_tx_buffer))
					{
						r = USB_common::USB_RESP::FAIL;
						break;
					}

					for(size_t i = 0; i < config_desc->size(); i++)
					{
						uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "CONFIGURATION - 0x%02X", m_tx_buffer.data()[i]);
					}

					//send iface and ep descriptors if asked for more
					if(req->wLength > config_desc->bLength)
					{					
						Descriptor_base const * desc_node = config_desc->get_desc_list().front<Descriptor_base>();

						while(desc_node)
						{
							uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "CONFIGURATION - node");

							if(m_tx_buffer.size() == req->wLength)
							{
								break;
							}

							if(!desc_node->serialize(&m_tx_buffer))
							{
								r = USB_common::USB_RESP::FAIL;
								break;
							}

							desc_node = desc_node->next<Descriptor_base>();
						}
					}

					r = USB_common::USB_RESP::ACK;
					break;
				}
				case USB_common::DESCRIPTOR_TYPE::STRING:
				{
					String_descriptor_zero::LANGID lang_idx = static_cast<String_descriptor_zero::LANGID>(req->wIndex);

					String_desc_table::String_desc_const_ptr string_desc = m_desc_table->get_string_descriptor(lang_idx, desc_index);
					if(!string_desc)
					{
						r = USB_common::USB_RESP::FAIL;
						break;
					}

					m_tx_buffer.reset();
					if(!string_desc->serialize(&m_tx_buffer))
					{
						r = USB_common::USB_RESP::FAIL;
						break;
					}

					r = USB_common::USB_RESP::ACK;
					break;
				}
				// case USB_common::DESCRIPTOR_TYPE::INTERFACE:
				// case USB_common::DESCRIPTOR_TYPE::ENDPOINT:
				default:
				{
					r = USB_common::USB_RESP::NAK;
					break;
				}
			}

			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_DESCRIPTOR:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_DESCRIPTOR");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		case Setup_packet::DEVICE_REQUEST::GET_CONFIGURATION:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "GET_CONFIGURATION");

			if(
				(req->wValue  != 0) ||
				(req->wIndex  != 0) ||
				(req->wLength != 1)
				)
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			uint8_t temp = 0;
			if(!get_configuration(&temp))
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			m_tx_buffer.insert(&temp, 1);
			r = USB_common::USB_RESP::ACK;
			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_CONFIGURATION:
		{
			uart1_log<64>(LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "SET_CONFIGURATION");
			if(
				(Byte_util::get_b1(req->wValue) != 0) ||
				(req->wIndex  != 0)                   ||
				(req->wLength != 0)
				)
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			const uint8_t bConfigurationValue = Byte_util::get_b0(req->wValue);
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
USB_common::USB_RESP USB_core::handle_std_iface_request(Setup_packet* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::INTERFACE_REQUEST>(req->bRequest))
	{
		case Setup_packet::INTERFACE_REQUEST::GET_STATUS:
		{
			m_tx_buffer.reset();

			std::array<uint8_t, 2> status;
			status.fill(0);

			m_tx_buffer.insert(status.data(), status.size());

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
USB_common::USB_RESP USB_core::handle_std_ep_request(Setup_packet* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	const Setup_packet::FEATURE_SELECTOR feature = static_cast<Setup_packet::FEATURE_SELECTOR>(req->wValue);
	if(feature != Setup_packet::FEATURE_SELECTOR::ENDPOINT_HALT)
	{
		return r;
	}

	const uint8_t endpoint_idx = Byte_util::get_b0(req->wIndex);

	switch(static_cast<Setup_packet::ENDPOINT_REQUEST>(req->bRequest))
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
			m_tx_buffer.reset();
			
			if(m_driver->ep_is_stalled(endpoint_idx))
			{			
				m_tx_buffer.insert((1U << 0));
			}
			else
			{
				m_tx_buffer.insert(0);
			}

			m_tx_buffer.insert(0);

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

	m_configuration = bConfigurationValue;

	//TODO move this to a child or callback
	{
		usb_driver_base::ep_cfg ep1;
		ep1.num = 0x01;
		ep1.size = 512;
		ep1.type = usb_driver_base::EP_TYPE::BULK;
		m_driver->ep_config(ep1);
	}
	{
		usb_driver_base::ep_cfg ep2;
		ep2.num = 0x80 | 0x01;
		ep2.size = 512;
		ep2.type = usb_driver_base::EP_TYPE::BULK;
		m_driver->ep_config(ep2);
	}
	{
		usb_driver_base::ep_cfg ep3;
		ep3.num = 0x80 | 0x02;
		ep3.size = 8;
		ep3.type = usb_driver_base::EP_TYPE::INTERRUPT;
		m_driver->ep_config(ep3);
	}
	return true;
}
bool USB_core::get_configuration(uint8_t* const bConfigurationValue)
{
	*bConfigurationValue = m_configuration;
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