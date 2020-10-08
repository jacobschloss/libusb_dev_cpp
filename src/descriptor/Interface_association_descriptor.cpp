/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2020 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/descriptor/Interface_association_descriptor.hpp"

#include "common_util/Byte_util.hpp"

constexpr uint8_t Interface_association_descriptor::bLength;
constexpr uint8_t Interface_association_descriptor::bDescriptorType;

bool Interface_association_descriptor::serialize(Interface_association_descriptor_array* const out_array) const
{
	(*out_array)[0] = bLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bFirstInterface;
	(*out_array)[3] = bInterfaceCount;
	(*out_array)[4] = bFunctionClass;
	(*out_array)[5] = bFunctionSubClass;
	(*out_array)[6] = bFunctionProtocol;
	(*out_array)[7] = iFunction;

	return true;
}
bool Interface_association_descriptor::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	Interface_association_descriptor_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());

	return true;
}
