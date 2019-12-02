/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/class/usb_class.hpp"

#include "libusb_dev_cpp/descriptor/Device_descriptor.hpp"
#include "libusb_dev_cpp/descriptor/Interface_descriptor.hpp"
#include "libusb_dev_cpp/descriptor/Endpoint_descriptor.hpp"
#include "libusb_dev_cpp/descriptor/Configuration_descriptor.hpp"

class CDC_class : public USB_class
{
public:

	CDC_class();
	~CDC_class() override;

	USB_common::USB_RESP handle_class_request(Setup_packet* const req, Buffer_adapter_rx* const buf_from_host, Buffer_adapter_tx* const buf_to_host) override;

	void process();

	// bool set_configuration(const uint8_t bConfigurationValue) override;
	// bool get_configuration(uint8_t* const bConfigurationValue) override;

protected:

};
