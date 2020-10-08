/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/descriptor/Endpoint_descriptor.hpp"

#include "common_util/Byte_util.hpp"

bool Endpoint_descriptor::serialize(Endpoint_descriptor_array* const out_array) const
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
bool Endpoint_descriptor::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	Endpoint_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

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

bool Endpoint_companion_descriptor::serialize(Endpoint_companion_descriptor_array* const out_array) const
{
	(*out_array)[0] = bLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bMaxBurst;
	(*out_array)[3] = bmAttributes;
	(*out_array)[4] = Byte_util::get_b0(wBytesPerInterval);
	(*out_array)[5] = Byte_util::get_b1(wBytesPerInterval);

	return true;
}
bool Endpoint_companion_descriptor::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	Endpoint_companion_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

	return true;
}

bool SSP_Isoc_companion_descriptor::serialize(SSP_Isoc_companion_descriptor_array* const out_array) const
{
	(*out_array)[0] = bLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = Byte_util::get_b0(wReserved);
	(*out_array)[3] = Byte_util::get_b1(wReserved);
	(*out_array)[4] = Byte_util::get_b0(dwBytesPerInterval);
	(*out_array)[5] = Byte_util::get_b1(dwBytesPerInterval);
	(*out_array)[6] = Byte_util::get_b2(dwBytesPerInterval);
	(*out_array)[7] = Byte_util::get_b3(dwBytesPerInterval);

	return true;
}
bool SSP_Isoc_companion_descriptor::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	SSP_Isoc_companion_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

	return true;
}
