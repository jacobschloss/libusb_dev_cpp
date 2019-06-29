/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/class/cdc/cdc_usb.hpp"

#include "libusb_dev_cpp/core/Get_descriptor.hpp"


CDC_usb::CDC_usb()
{

}
CDC_usb::~CDC_usb()
{

}

bool CDC_usb::fill_descriptors()
{
	Device_descriptor dev_desc;
	dev_desc.bcdUSB = USB_common::build_bcd(2, 0, 0);
	dev_desc.bDeviceClass    = 0;
	dev_desc.bDeviceSubClass = 0;
	dev_desc.bDeviceProtocol = 0;
	dev_desc.bMaxPacketSize0 = 8;
	dev_desc.idVendor  = 0x0123;
	dev_desc.idProduct = 0x4567;
	dev_desc.bcdDevice = USB_common::build_bcd(1, 0, 0);
	dev_desc.iManufacturer      = 0;
	dev_desc.iProduct           = 0;
	dev_desc.iSerialNumber      = 0;
	dev_desc.bNumConfigurations = 1;
	if(!dev_desc.serialize(&dev_desc_array))
	{
		return false;
	}

	// Configuration_descriptor conf_desc;
	// conf_desc.wTotalLength        = Configuration_descriptor::bLength;
	// conf_desc.bNumInterfaces      = 2;
	// conf_desc.bConfigurationValue = 1;
	// conf_desc.iConfiguration      = 0;
	// conf_desc.bmAttributes        = 0;
	// conf_desc.bMaxPower           = Configuration_descriptor::ma_to_maxpower(150);
	// if(!conf_desc.serialize(&conf_desc_array))
	// {
	// 	return false;
	// }

	Interface_descriptor iface_desc;
	if(!iface_desc.serialize(&iface_desc_array))
	{
		return false;
	}

	Endpoint_descriptor ep_in_desc;
	if(!ep_in_desc.serialize(&ep_in_desc_array))
	{
		return false;
	}

	Endpoint_descriptor ep_out_desc;
	if(!ep_out_desc.serialize(&ep_out_desc_array))
	{
		return false;
	}

	return true;
}

USB_common::USB_RESP CDC_usb::handle_std_device_request(Setup_packet* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::DEVICE_REQUEST>(req->bRequest))
	{
		case Setup_packet::DEVICE_REQUEST::GET_DESCRIPTOR:
		{
			Get_descriptor get_desc;
			if(!get_desc.deserialize(req->wValue))
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}
			const uint16_t lang_id = req->wIndex;

			switch(get_desc.type)
			{
				case Get_descriptor::DESCRIPTOR_TYPES::DEVICE:
				{

				}
				case Get_descriptor::DESCRIPTOR_TYPES::CONFIGURATION:
				{
					
				}
				case Get_descriptor::DESCRIPTOR_TYPES::STRING:
				{
					
				}
				default:
				{
					r = USB_common::USB_RESP::FAIL;
					break;
				}
			}

			break;
		}
		default:
		{
			//try the base class
			r = USB_core::handle_std_device_request(req);
			break;
		}
	}

	return r;
}
USB_common::USB_RESP CDC_usb::handle_std_iface_request(Setup_packet* const req)
{
	return USB_common::USB_RESP::FAIL;
}
USB_common::USB_RESP CDC_usb::handle_std_ep_request(Setup_packet* const req)
{
	return USB_common::USB_RESP::FAIL;
}