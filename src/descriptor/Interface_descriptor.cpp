#include "libusb_dev_cpp/descriptor/Interface_descriptor.hpp"

bool Interface_descriptor::serialize(Interface_descriptor_array* const out_array)
{
	(*out_array)[0] = bLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bInterfaceNumber;
	(*out_array)[3] = bAlternateSetting;
	(*out_array)[4] = bNumEndpoints;
	(*out_array)[5] = bInterfaceClass;
	(*out_array)[6] = bInterfaceSubClass;
	(*out_array)[7] = bInterfaceProtocol;
	(*out_array)[8] = iInterface;

	return true;
}
bool Interface_descriptor::deserialize(const Interface_descriptor_array& array)
{
	if(bLength != array[0])
	{
		return false;
	}
	if(bDescriptorType != array[1])
	{
		return false;
	}

	bInterfaceNumber   = array[2];
	bAlternateSetting  = array[3];
	bNumEndpoints      = array[4];
	bInterfaceClass    = array[5];
	bInterfaceSubClass = array[6];
	bInterfaceProtocol = array[7];
	iInterface         = array[8];

	return true;
}
