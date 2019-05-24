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
	if(!dev_desc.serialize(&dev_desc_array))
	{
		return false;
	}

	Configuration_descriptor conf_desc;
	if(!conf_desc.serialize(&conf_desc_array))
	{
		return false;
	}

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

USB_common::USB_RESP CDC_usb::handle_device_request(Control_request* const req)
{
	USB_common::USB_RESP r = USB_common::USB_RESP::FAIL;

	switch(static_cast<Setup_packet::DEVICE_REQUEST>(req->setup_packet.bRequest))
	{
		case Setup_packet::DEVICE_REQUEST::GET_DESCRIPTOR:
		{
			Get_descriptor get_desc;
			if(!get_desc.deserialize(req->setup_packet.wValue))
			{
				r = USB_common::USB_RESP::FAIL;
				break;
			}
			const uint16_t lang_id = req->setup_packet.wIndex;

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
			r = USB_core::handle_device_request(req);
			break;
		}
	}

	return r;
}
USB_common::USB_RESP CDC_usb::handle_iface_request(Control_request* const req)
{
	return USB_common::USB_RESP::FAIL;
}
USB_common::USB_RESP CDC_usb::handle_ep_request(Control_request* const req)
{
	return USB_common::USB_RESP::FAIL;
}