/**
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/class/cdc/cdc_desc.hpp"

#include "common_util/Byte_util.hpp"

namespace CDC
{

bool CDC_header_descriptor::serialize(CDC_header_descriptor_array* const out_array) const
{
	(*out_array)[0] = bFunctionLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bDescriptorSubType;
	(*out_array)[3] = Byte_util::get_b0(bcdCDC);
	(*out_array)[4] = Byte_util::get_b1(bcdCDC);

	return true;
}
bool CDC_header_descriptor::serialize(Buffer_adapter* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	CDC_header_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

	return true;
}
bool CDC_call_management_descriptor::serialize(CDC_call_management_descriptor_array* const out_array) const
{
	(*out_array)[0] = bFunctionLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bDescriptorSubType;
	(*out_array)[3] = bmCapabilities;
	(*out_array)[4] = bDataInterface;

	return true;
}
bool CDC_call_management_descriptor::serialize(Buffer_adapter* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	CDC_call_management_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

	return true;
}
bool CDC_acm_descriptor::serialize(CDC_acm_descriptor_array* const out_array) const
{
	(*out_array)[0] = bFunctionLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bDescriptorSubType;
	(*out_array)[3] = bmCapabilities;

	return true;
}
bool CDC_acm_descriptor::serialize(Buffer_adapter* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	CDC_acm_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

	return true;
}
bool CDC_union_descriptor::serialize(CDC_union_descriptor_array* const out_array) const
{
	(*out_array)[0] = bFunctionLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bDescriptorSubtype;
	(*out_array)[3] = bMasterInterface;
	(*out_array)[4] = bSlaveInterface0;

	return true;
}
bool CDC_union_descriptor::serialize(Buffer_adapter* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	CDC_union_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

	return true;
}

}