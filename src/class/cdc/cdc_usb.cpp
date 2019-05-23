#include "libusb_dev_cpp/class/cdc/cdc_usb.hpp"

#include "libusb_dev_cpp/core/Get_descriptor.hpp"


CDC_usb::CDC_usb()
{

}
CDC_usb::~CDC_usb()
{

}

USB_common::USB_RESP CDC_usb::handle_device_request(Control_request* const req)
{
	Get_descriptor get_desc;
	get_desc.deserialize(req->setup_packet.wIndex);

	return USB_common::USB_RESP::FAIL;
}
USB_common::USB_RESP CDC_usb::handle_iface_request(Control_request* const req)
{
	return USB_common::USB_RESP::FAIL;
}
USB_common::USB_RESP CDC_usb::handle_ep_request(Control_request* const req)
{
	return USB_common::USB_RESP::FAIL;
}