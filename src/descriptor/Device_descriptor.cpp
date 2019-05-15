#include "libusb_dev_cpp/descriptor/Device_descriptor.hpp"

#include "common_util/Byte_util.hpp"

bool Device_descriptor::serialize(Device_descriptor_array* const out_array)
{
	(*out_array)[0]  = bLength;
	(*out_array)[1]  = bDescriptorType;
	(*out_array)[2]  = bcdUSB[0];
	(*out_array)[3]  = bcdUSB[1];
	(*out_array)[4]  = bDeviceClass;
	(*out_array)[5]  = bDeviceSubClass;
	(*out_array)[6]  = bDeviceProtocol;
	(*out_array)[7]  = bMaxPacketSize;
	(*out_array)[8]  = Byte_util::get_b0(idVendor);
	(*out_array)[9]  = Byte_util::get_b1(idVendor);
	(*out_array)[10] = Byte_util::get_b0(idProduct);
	(*out_array)[11] = Byte_util::get_b1(idProduct);
	(*out_array)[12] = bcdDevice[0];
	(*out_array)[13] = bcdDevice[1];
	(*out_array)[14] = iManufacturer;
	(*out_array)[15] = iProduct;
	(*out_array)[16] = iSerialNumber;
	(*out_array)[17] = bNumConfigurations;

	return true;
}
bool Device_descriptor::deserialize(const Device_descriptor_array& array)
{
	if(bLength != array[0])
	{
		return false;
	}
	if(bDescriptorType != array[1])
	{
		return false;
	}

	bcdUSB[0]          = array[2];
	bcdUSB[1]          = array[3];
	bDeviceClass       = array[4];
	bDeviceSubClass    = array[5];
	bDeviceProtocol    = array[6];
	bMaxPacketSize     = array[7];
	idVendor           = Byte_util::make_u16(array[9], array[8]);
	idProduct          = Byte_util::make_u16(array[11], array[10]);
	bcdDevice[0]       = array[12];
	bcdDevice[1]       = array[13];
	iManufacturer      = array[14];
	iProduct           = array[15];
	iSerialNumber      = array[16];
	bNumConfigurations = array[17];

	return true;
}