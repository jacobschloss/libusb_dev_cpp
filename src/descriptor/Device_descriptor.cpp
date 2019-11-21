/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/descriptor/Device_descriptor.hpp"

#include "common_util/Byte_util.hpp"

bool Device_descriptor::serialize(Device_descriptor_array* const out_array) const
{
	(*out_array)[0]  = bLength;
	(*out_array)[1]  = bDescriptorType;
	(*out_array)[2]  = Byte_util::get_b0(bcdUSB);
	(*out_array)[3]  = Byte_util::get_b1(bcdUSB);
	(*out_array)[4]  = bDeviceClass;
	(*out_array)[5]  = bDeviceSubClass;
	(*out_array)[6]  = bDeviceProtocol;
	(*out_array)[7]  = bMaxPacketSize0;
	(*out_array)[8]  = Byte_util::get_b0(idVendor);
	(*out_array)[9]  = Byte_util::get_b1(idVendor);
	(*out_array)[10] = Byte_util::get_b0(idProduct);
	(*out_array)[11] = Byte_util::get_b1(idProduct);
	(*out_array)[12] = Byte_util::get_b0(bcdDevice);
	(*out_array)[13] = Byte_util::get_b1(bcdDevice);
	(*out_array)[14] = iManufacturer;
	(*out_array)[15] = iProduct;
	(*out_array)[16] = iSerialNumber;
	(*out_array)[17] = bNumConfigurations;

	return true;
}
bool Device_descriptor::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	Device_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

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

	bcdUSB             = Byte_util::make_u16(array[3], array[2]);
	bDeviceClass       = array[4];
	bDeviceSubClass    = array[5];
	bDeviceProtocol    = array[6];
	bMaxPacketSize0     = array[7];
	idVendor           = Byte_util::make_u16(array[9], array[8]);
	idProduct          = Byte_util::make_u16(array[11], array[10]);
	bcdDevice          = Byte_util::make_u16(array[13], array[12]);
	iManufacturer      = array[14];
	iProduct           = array[15];
	iSerialNumber      = array[16];
	bNumConfigurations = array[17];

	return true;
}