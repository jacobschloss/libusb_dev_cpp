/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/class/cdc/cdc_usb.hpp"

#include "libusb_dev_cpp/class/cdc/cdc.hpp"
#include "libusb_dev_cpp/class/cdc/cdc_mgmt_requests.hpp"

#include "freertos_cpp_util/logging/Global_logger.hpp"

using freertos_util::logging::Global_logger;
using freertos_util::logging::LOG_LEVEL;

CDC_class::CDC_class()
{

}
CDC_class::~CDC_class()
{

}

void CDC_class::process()
{
	
}

USB_common::USB_RESP CDC_class::handle_class_request(Setup_packet* const req, Buffer_adapter_rx* const buf_from_host, Buffer_adapter_tx* const buf_to_host)
{
	freertos_util::logging::Logger* const logger = freertos_util::logging::Global_logger::get();
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	buf_to_host->reset();

	// Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.bRequest: 0x%02X",
	// 	m_setup_packet.bRequest
	// 	);

	// Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.wValue: 0x%04X",
	// 	m_setup_packet.wValue
	// 	);

	// Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.wIndex: 0x%04X",
	// 	m_setup_packet.wIndex
	// 	);

	// Global_logger::get()->log(freertos_util::logging::LOG_LEVEL::DEBUG, "USB_core", "setup_packet.wLength: 0x%04X",
	// 	m_setup_packet.wLength
	// 	);

	if(req->wLength != buf_from_host->size())
	{
		logger->log(LOG_LEVEL::WARN, "CDC_class", "handle_class_request: buffer length and req wLength and do not match: %u/%u", buf_from_host->size(), req->wLength);
	}

	logger->log(LOG_LEVEL::DEBUG, "CDC_class", "handle_class_request: buf_from_host %d", buf_from_host->size());
	for(size_t i = 0; i < buf_from_host->size(); i++)
	{
		logger->log(LOG_LEVEL::DEBUG, "CDC_class", "\tbuf_from_host[%u]: 0x%02X", i, buf_from_host->data()[i]);
	}

	switch(static_cast<CDC::CDC_REQUESTS>(req->bRequest))
	{
		case CDC::CDC_REQUESTS::CLEAR_COMM_FEATURE:
		{
			logger->log(LOG_LEVEL::INFO, "CDC_class", "handle_class_request: CLEAR_COMM_FEATURE");
			break;
		}
		case CDC::CDC_REQUESTS::SET_LINE_CODING:
		{
			logger->log(LOG_LEVEL::INFO, "CDC_class", "handle_class_request: SET_LINE_CODING");

			CDC::LINE_CODING line;
			if(line.deserialize(buf_from_host))
			{
				r = USB_common::USB_RESP::ACK;
			}
			else
			{
				// r = USB_common::USB_RESP::NAK;
				r = USB_common::USB_RESP::ACK;
				logger->log(LOG_LEVEL::INFO, "CDC_class::SET_LINE_CODING", "deserialize failed");
			}

			break;
		}
		case CDC::CDC_REQUESTS::GET_LINE_CODING:
		{
			logger->log(LOG_LEVEL::INFO, "CDC_class", "handle_class_request: GET_LINE_CODING");
			
			CDC::LINE_CODING line;

			line.dwDTERRate = 9600;
			line.bCharFormat = static_cast<uint8_t>(CDC::LINE_CODING::CHAR_FORMAT::ONE_STOP);
			line.bParityType = static_cast<uint8_t>(CDC::LINE_CODING::PARITY_TYPE::NONE);
			line.bDataBits   = static_cast<uint8_t>(CDC::LINE_CODING::DATA_BITS::EIGHT);

			if(!line.serialize(buf_to_host))
			{
				logger->log(LOG_LEVEL::ERROR, "CDC_class", "handle_class_request: GET_LINE_CODING CDC::LINE_CODING ser failed");	
			}

			r = USB_common::USB_RESP::ACK;
			break;
		}
		case CDC::CDC_REQUESTS::SET_CONTROL_LINE_STATE:
		{
			logger->log(LOG_LEVEL::INFO, "CDC_class", "handle_class_request: SET_CONTROL_LINE_STATE");
			CDC::SET_CONTROL_LINE_STATE ctrl_line_state;
			ctrl_line_state.wValue = req->wValue;

			logger->log(LOG_LEVEL::INFO, "CDC_class", "handle_class_request: SET_CONTROL_LINE_STATE, DTR: %d, RTR: %d", ctrl_line_state.DTR(), ctrl_line_state.RTS());

			r = USB_common::USB_RESP::ACK;
			break;
		}
		case CDC::CDC_REQUESTS::SEND_BREAK:
		{
			logger->log(LOG_LEVEL::INFO, "CDC_class", "handle_class_request: SEND_BREAK");
			r = USB_common::USB_RESP::NAK;
			break;
		}
		default:
		{
			logger->log(LOG_LEVEL::INFO, "CDC_class", "handle_class_request: unknown, %d", int(req->bRequest));
			break;	
		}
	}
	
	return r;
}
