#include "libusb_dev_cpp/descriptor/Endpoint_descriptor.hpp"

#include "common_util/Byte_util.hpp"

bool Endpoint_descriptor::serialize(Endpoint_descriptor_array* const out_array)
{
	(*out_array)[0] = bLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bEndpointAddress;
	(*out_array)[3] = bmAttributes;
	(*out_array)[4] = Byte_util::get_b0(wMaxPacketSize);
	(*out_array)[5] = Byte_util::get_b1(wMaxPacketSize);
	(*out_array)[6] = bInterval;

	return true;
}
bool Endpoint_descriptor::deserialize(const Endpoint_descriptor_array& array)
{
	if(bLength != array[0])
	{
		return false;
	}
	if(bDescriptorType != array[1])
	{
		return false;
	}

	bEndpointAddress = array[2];
	bmAttributes     = array[3];
	wMaxPacketSize   = Byte_util::make_u16(array[5], array[4]);
	bInterval        = array[6];

	return true;
}
