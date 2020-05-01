/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/core/usb_common.hpp"

#include "libusb_dev_cpp/core/Setup_packet.hpp"

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

class USB_class
{
public:

	USB_class();
	virtual ~USB_class();

	virtual USB_common::USB_RESP handle_class_request(Setup_packet* const req, Buffer_adapter_rx* const buf_from_host, Buffer_adapter_tx* const buf_to_host) = 0;

protected:
};