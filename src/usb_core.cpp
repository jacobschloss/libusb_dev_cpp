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

#include "freertos_cpp_util/logging/Global_logger.hpp"

#include <algorithm>
#include <functional>

using freertos_util::logging::Global_logger;

USB_core::USB_core()
{
	m_driver = nullptr;
	m_usb_class = nullptr;
	m_desc_table = nullptr;
}

USB_core::~USB_core()
{
	
}

bool USB_core::initialize(usb_driver_base* const driver, const uint8_t ep0size, const Buffer_adapter_tx& tx_buf, const Buffer_adapter_rx& rx_buf)
{
	m_address = 0;
	m_configuration = 0;

	m_driver = driver;

	m_tx_buffer = tx_buf;
	m_tx_buffer.reset();
	m_rx_buffer = rx_buf;
	m_rx_buffer.reset();

	m_set_config_callback_ctx = nullptr;
	m_set_config_callback_func = nullptr;

	m_usb_core_handle_event = std::bind(&USB_core::handle_event, this, std::placeholders::_1, std::placeholders::_2);

	return true;
}

void USB_core::set_usb_class(USB_class* const usb_class)
{
	m_usb_class = usb_class;
}

void USB_core::set_descriptor_table(Descriptor_table* const desc_table)
{
	m_desc_table = desc_table;
}

bool USB_core::poll_driver()
{
	m_driver->poll(m_usb_core_handle_event);
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
	ep0.size = 8;
	ep0.type = usb_driver_base::EP_TYPE::CONTROL;
	m_driver->ep_config(ep0);

	return true;
}

bool USB_core::handle_enum_done()
{
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

bool USB_core::handle_sof()
{
	return true;
}

bool USB_core::wait_event_loop()
{
	bool ret = poll_event_loop(true);
	// if(!ret)
	// {
	// 	Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "wait_event_loop ret false");
	// }

	return ret;
}

bool USB_core::poll_event_loop()
{
	return poll_event_loop(false);
}

bool USB_core::poll_event_loop(const bool wait)
{
	usb_core_event core_evt;

	if(!m_event_queue.pop_front_wait(&core_evt, wait))
	{
		return false;
	}

	const uint8_t ep_addr = USB_common::get_ep_addr(core_evt.ep);

	USB_common::Event_callback func = nullptr;
	bool ret = false;
	switch(core_evt.event)
	{
		case USB_common::USB_EVENTS::RESET:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "USB_EVENTS::RESET");

			ret = handle_reset();
			break;
		}
		case USB_common::USB_EVENTS::ENUM_DONE:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "USB_EVENTS::ENUM_DONE");

			ret = handle_enum_done();
			break;
		}
		case USB_common::USB_EVENTS::EP_RX:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::TRACE, "USB_core", "USB_EVENTS::EP_RX");

			func = m_driver->get_ep_rx_callback(ep_addr);
			if(func)
			{
				func(core_evt.event, core_evt.ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::EP_TX:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::TRACE, "USB_core", "USB_EVENTS::EP_TX");

			func = m_driver->get_ep_tx_callback(ep_addr);
			if(func)
			{
				func(core_evt.event, core_evt.ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::CTRL_SETUP_PHASE_DONE:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "USB_EVENTS::CTRL_SETUP_PHASE_DONE");

			func = m_driver->get_ep_setup_callback(ep_addr);
			if(func)
			{
				func(core_evt.event, core_evt.ep);
			}
			break;
		}
		case USB_common::USB_EVENTS::CTRL_DATA_PHASE_DONE:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "USB_EVENTS::CTRL_DATA_PHASE_DONE");

			break;
		}
		case USB_common::USB_EVENTS::EARLY_SUSPEND:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "USB_EVENTS::EARLY_SUSPEND");

			//we will suspend soon
			break;
		}
		case USB_common::USB_EVENTS::SUSPEND:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "USB_EVENTS::SUSPEND");

			//we are suspended
			break;
		}
		case USB_common::USB_EVENTS::SOF:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::TRACE, "USB_core", "USB_EVENTS::SOF");

			ret = handle_sof();
			break;
		}
		case USB_common::USB_EVENTS::NONE:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::WARN, "USB_core", "USB_EVENTS::NONE");

			//ISR triggered but we don't care
			break;
		}
		default:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core", "Unknown event");
			break;
		}
	}

	func = m_driver->get_event_callback(ep_addr);//todo use the get addr helper func
	if(func)
	{
		func(core_evt.event, core_evt.ep);
		ret = true;
	}

	return ret;
}

bool USB_core::handle_event(const USB_common::USB_EVENTS evt, const uint8_t ep)
{
	usb_core_event core_evt;
	core_evt.event = evt;
	core_evt.ep = ep;

	bool ret = false;

	//verify if this is really an interrupt
	//in some cases eg the USB library will have a code path that is optionally polled or ISR
	if(xPortIsInsideInterrupt() == pdFALSE)
	{
		ret = m_event_queue.push_back(core_evt);
	}
	else
	{
		ret = m_event_queue.push_back_isr(core_evt);
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

bool USB_core::handle_ep0_rx(const USB_common::USB_EVENTS event, const uint8_t ep)
{
	switch(m_control_state)
	{
		case USB_CONTROL_STATE::IDLE:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "handle_ep0_rx USB_CONTROL_STATE::IDLE");

			const Setup_packet::Setup_packet_array* setup_packet_array = m_driver->get_last_setup_packet();

			//decode
			m_setup_packet = Setup_packet();
			if(!m_setup_packet.deserialize(*setup_packet_array))
			{
				return false;
			}

			Request_type req_type;
			if(!m_setup_packet.get_request_type(&req_type))
			{
				return false;
			}

			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.bmRequestType: 0x%02X",
				m_setup_packet.bmRequestType
				);

			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.bRequest: 0x%02X",
				m_setup_packet.bRequest
				);

			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.wValue: 0x%04X",
				m_setup_packet.wValue
				);

			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.wIndex: 0x%04X",
				m_setup_packet.wIndex
				);

			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.wLength: 0x%04X",
				m_setup_packet.wLength
				);

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
					return true;
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
				return true;
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
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "handle_ep0_rx USB_CONTROL_STATE::RXDATA");

			EP_buffer_mgr_base* ep0_buf_mgr = m_driver->get_ep0_buffer();

			Buffer_adapter_base* ep0_buf = ep0_buf_mgr->poll_dequeue_buffer(0);
			if(ep0_buf)
			{
				if(m_rx_buffer.rem_len < ep0_buf->size())
				{
					//we got too much data, that is weird
					stall_control_ep(ep);
				
					//release ep buffer
					ep0_buf_mgr->release_buffer(0, ep0_buf);
					ep0_buf = nullptr;

					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "handle_ep0_rx USB_CONTROL_STATE::RXDATA too much data");
					return true;
				}

				//copy
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "CDC_class", "handle_ep0_rx: ep0_buf %d", ep0_buf->size());
				for(size_t i = 0; i < ep0_buf->size(); i++)
				{
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "CDC_class", "\tep0_buf[%u]: 0x%02X", i, ep0_buf->data()[i]);
				}

				const size_t to_copy    = std::min(ep0_buf->size(), m_rx_buffer.rem_len);
				const size_t num_copied = m_rx_buffer.insert(ep0_buf->data(), to_copy);

				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "handle_ep0_rx USB_CONTROL_STATE::RXDATA got %u", num_copied);

				//release ep buffer
				ep0_buf_mgr->release_buffer(0, ep0_buf);
				ep0_buf = nullptr;

				if(m_rx_buffer.rem_len > 0)
				{
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "handle_ep0_rx USB_CONTROL_STATE::RXDATA keep reading, have %u, want %u", m_rx_buffer.size(), m_rx_buffer.rem_len);
					//keep reading
					//skip evt processing
					return true;
				}
			}
			else
			{
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core", "handle_ep0_rx USB_CONTROL_STATE::RXDATA ep0 did not have buffer");
			}
			break;
		}
		case USB_CONTROL_STATE::STATUS_OUT:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "handle_ep0_rx USB_CONTROL_STATE::STATUS_OUT");

			//handle status out packet
			m_rx_buffer.reset();
			
			m_control_state = USB_CONTROL_STATE::IDLE;
			if(m_setup_complete_callback)
			{
				m_setup_complete_callback();
			}
			return true;
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
		return false;
	}

	switch(process_request(&m_setup_packet))
	{
		case USB_common::USB_RESP::ACK:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core", "handle_ep0_rx process_request - ACK");

			//did the host ask us to send data? if so, send it
			if((req_type.data_dir == Request_type::DATA_DIR::DEV_TO_HOST))
			{
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core", "handle_ep0_rx process_request - ACK/%u", m_setup_packet.wLength);
				if(m_tx_buffer.rem_len >= m_setup_packet.wLength)
				{
					m_tx_buffer.rem_len = m_setup_packet.wLength;
				}

				if(m_tx_buffer.rem_len != m_setup_packet.wLength)
				{
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core", "handle_ep0_rx process_request - m_tx_buffer too small, %u/%u", m_tx_buffer.rem_len, m_setup_packet.wLength);
				}

				m_control_state = USB_CONTROL_STATE::TXDATA;
				handle_ep0_tx(event, ep | 0x80);
			}
			else
			{
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core", "handle_ep0_rx process_request - ACK/zlp");

				//otherwise send a zlp status packet
				m_tx_buffer.reset();
				m_driver->ep_write(ep | 0x80, 0, 0);
				m_control_state = USB_CONTROL_STATE::STATUS_IN;
			}
			break;
		}
		case USB_common::USB_RESP::NAK:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core", "handle_ep0_rx process_request - NAK");

			m_control_state = USB_CONTROL_STATE::STATUS_IN;
			break;
		}
		case USB_common::USB_RESP::FAIL:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core", "handle_ep0_rx process_request - FAIL");

			//force a NAK to reset the state machine, probably best bet of reseting
			m_control_state = USB_CONTROL_STATE::STATUS_IN;
			break;
		}
		default:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::FATAL, "USB_core", "handle_ep0_rx process_request - default");
			//invalid state, reset control endpoint
			stall_control_ep(ep);
			break;
		}
	}
	
	return true;
}
bool USB_core::handle_ep0_tx(const USB_common::USB_EVENTS event, const uint8_t ep)
{
	switch(m_control_state)
	{
		case USB_CONTROL_STATE::TXDATA:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core", "USB_CONTROL_STATE::TXDATA");

			const size_t ep0size = m_driver->get_ep0_config().size;
			const size_t num_to_write = std::min(m_tx_buffer.rem_len, ep0size);

			int num_wrote = m_driver->ep_write(ep | 0x80, m_tx_buffer.curr_ptr, num_to_write);

			if(num_wrote < 0)
			{
				// Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "ep_write error");
			}
			else
			{
				m_tx_buffer.curr_ptr += num_wrote;
				m_tx_buffer.rem_len  -= num_wrote;
				// Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "wrote %d, left %d", num_wrote, m_tx_buffer.rem_len);
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
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core", "USB_CONTROL_STATE::TXZLP");

			const int ret = m_driver->ep_write(ep | 0x80, nullptr, 0);
			if(ret != 0)
			{
				// Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "TXZLP had error on ep_write");
			}

			m_control_state = USB_CONTROL_STATE::TXCOMP;
			break;
		}
		case USB_CONTROL_STATE::TXCOMP:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core", "USB_CONTROL_STATE::TXCOMP");

			m_control_state = USB_CONTROL_STATE::STATUS_OUT;
			break;	
		}
		case USB_CONTROL_STATE::STATUS_IN:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core", "USB_CONTROL_STATE::STATUS_IN");

			m_control_state = USB_CONTROL_STATE::IDLE;
			//tx complete, so status in ack sent
			//call the deffered process callback
			if(m_setup_complete_callback)
			{
				m_setup_complete_callback();
			}
			return true;
		}
		default:
		{
			// Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core::handle_ep_tx", "default, event %d, state %d", event, m_control_state);
			break;
		}
	}

	return true;
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
			switch(request_type.recipient)
			{
				case Request_type::RECIPIENT::DEVICE:
				{
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::process_request", "STANDARD DEVICE request");
					r = handle_std_device_request(req);
					break;
				}
				case Request_type::RECIPIENT::INTERFACE:
				{
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::process_request", "STANDARD INTERFACE request");
					r = handle_std_iface_request(req);
					break;
				}
				case Request_type::RECIPIENT::ENDPOINT:
				{
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::process_request", "STANDARD ENDPOINT request");
					r = handle_std_ep_request(req);
					break;
				}
				default:
				{
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core::process_request", "STANDARD request was not device, interface, or endpoint");
					r = USB_common::USB_RESP::FAIL;
					break;
				}
			}
			break;
		}
		case Request_type::TYPE::CLASS:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::process_request", "CLASS request, m_rx_buffer has %u", m_rx_buffer.size());
			if(m_usb_class)
			{
				r = m_usb_class->handle_class_request(req, &m_rx_buffer, &m_tx_buffer);
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::process_request", "CLASS request, m_tx_buffer has %u", m_tx_buffer.size());
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::process_request", "CLASS request, m_tx_buffer rem_len %u", m_tx_buffer.rem_len);
			}
			else
			{
				r = USB_common::USB_RESP::FAIL;
			}
			break;
		}
		case Request_type::TYPE::VENDOR:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::process_request", "VENDOR request");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		case Request_type::TYPE::RESERVED:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::process_request", "RESERVED request");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		default:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::FATAL, "USB_core::process_request", "Unknown request, %d", int(request_type.type));

			r = USB_common::USB_RESP::FAIL;
			break;
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
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "GET_STATUS");
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
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "CLEAR_FEATURE");
			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_FEATURE:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "SET_FEATURE");
			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_ADDRESS:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "SET_ADDRESS");

			if((req->wIndex != 0) || (req->wLength != 0))
			{
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core::handle_std_device_request", "SET_ADDRESS packet invalid");
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			if(req->wValue > 127)
			{
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core::handle_std_device_request", "SET_ADDRESS address invalid");
				r = USB_common::USB_RESP::FAIL;
				break;
			}

			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::TRACE, "USB_core::handle_std_device_request", "Queue SET_ADDRESS to %d", req->wValue);
			
			// m_address = req->wValue;
			// m_setup_complete_callback = std::bind(&USB_core::set_address, this, req->wValue);
			set_address(req->wValue);
			r = USB_common::USB_RESP::ACK;
			break;
		}
		// handled by child class
		case Setup_packet::DEVICE_REQUEST::GET_DESCRIPTOR:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "GET_DESCRIPTOR");

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
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "GET_DESCRIPTOR - CONFIGURATION");

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
						Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "CONFIGURATION - 0x%02X", m_tx_buffer.data()[i]);
					}

					//send iface and ep descriptors if asked for more
					if(req->wLength > config_desc->bLength)
					{					
						Descriptor_base const * desc_node = config_desc->get_desc_list().front<Descriptor_base>();

						while(desc_node)
						{
							Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core::handle_std_device_request", "CONFIGURATION - node");

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
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "GET_DESCRIPTOR - STRING");

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
					Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core::handle_std_device_request", "GET_DESCRIPTOR - invalid type");

					r = USB_common::USB_RESP::NAK;
					break;
				}
			}

			break;
		}
		case Setup_packet::DEVICE_REQUEST::SET_DESCRIPTOR:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "SET_DESCRIPTOR");
			r = USB_common::USB_RESP::FAIL;
			break;
		}
		case Setup_packet::DEVICE_REQUEST::GET_CONFIGURATION:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "GET_CONFIGURATION");

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
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "SET_CONFIGURATION");
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
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core::handle_std_device_request", "Unknown request");
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
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_device_request", "GET_STATUS");

			m_tx_buffer.reset();

			std::array<uint8_t, 2> status;
			status.fill(0);

			m_tx_buffer.insert(status.data(), status.size());

			r = USB_common::USB_RESP::ACK;
			break;
		}
		default:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core::handle_std_iface_request", "Unknown request");

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
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_ep_request", "SET_FEATURE");
			m_driver->ep_stall(endpoint_idx);
			r = USB_common::USB_RESP::ACK;
			break;
		}
		case Setup_packet::ENDPOINT_REQUEST::CLEAR_FEATURE:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_ep_request", "CLEAR_FEATURE");
			if(req->wValue == 0x00)
			{
				m_driver->ep_unstall(endpoint_idx);
				r = USB_common::USB_RESP::ACK;
			}
			else
			{
				r = USB_common::USB_RESP::NAK;
			}
			break;
		}
		case Setup_packet::ENDPOINT_REQUEST::GET_STATUS:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_ep_request", "GET_STATUS");
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
			break;
		}
		default:
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core::handle_std_ep_request", "Unknown request %d", req->bRequest);
			r = USB_common::USB_RESP::FAIL;
			break;
		}
	}
	return r;
}

bool USB_core::set_configuration(const uint8_t bConfigurationValue)
{
	bool ret = false;

	if(m_set_config_callback_func)
	{
		if(m_set_config_callback_func(m_set_config_callback_ctx, bConfigurationValue))
		{
			m_configuration = bConfigurationValue;
			ret = true;

			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::INFO, "USB_core::set_configuration", "Config set to %d ok", m_configuration);
		}
		else
		{
			Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core::set_configuration", "Config set to %d failed, trying to set config to 0", bConfigurationValue);
			
			if(m_set_config_callback_func(m_set_config_callback_ctx, 0))
			{
				m_configuration = 0;
				ret = true;

				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::ERROR, "USB_core::set_configuration", "Config set to %d ok", m_configuration);
			}
			else
			{
				Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::FATAL, "USB_core::set_configuration", "Could not set configuration to 0");
			}

			ret = false;
		}
	}
	else
	{
		Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::FATAL, "USB_core::set_configuration", "No set configuration handler registered, can't configure");
		ret = false;
	}

	return ret;
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