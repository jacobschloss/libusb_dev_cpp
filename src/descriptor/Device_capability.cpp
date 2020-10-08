/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2020 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/descriptor/Device_capability.hpp"

#include "common_util/Byte_util.hpp"

constexpr uint8_t Device_capability_descriptor::bDescriptorType;

constexpr uint8_t Device_capability_usb20exension::bLength;
constexpr uint8_t Device_capability_usb20exension::bDevCapabilityType;

bool Device_capability_usb20exension::serialize(Device_capability_usb20exension_array* const out_array) const
{
	(*out_array)[0] = bLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bDevCapabilityType;
	(*out_array)[3] = Byte_util::get_b0(bmAttributes);
	(*out_array)[4] = Byte_util::get_b1(bmAttributes);
	(*out_array)[5] = Byte_util::get_b2(bmAttributes);
	(*out_array)[6] = Byte_util::get_b3(bmAttributes);

	return true;
}
bool Device_capability_usb20exension::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	Device_capability_usb20exension_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());
	return true;
}

constexpr uint8_t Device_capability_superspeed::bLength;
constexpr uint8_t Device_capability_superspeed::bDevCapabilityType;

bool Device_capability_superspeed::serialize(Device_capability_superspeed_array* const out_array) const
{
	(*out_array)[0] = bLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bDevCapabilityType;
	(*out_array)[3] = bmAttributes;
	(*out_array)[4] = Byte_util::get_b0(wSpeedsSupported);
	(*out_array)[5] = Byte_util::get_b1(wSpeedsSupported);
	(*out_array)[6] = bFunctionalitySupport;
	(*out_array)[7] = bU1DevExitLat;
	(*out_array)[8] = Byte_util::get_b0(wU2DevExitLat);
	(*out_array)[9] = Byte_util::get_b1(wU2DevExitLat);

	return true;
}
bool Device_capability_superspeed::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	Device_capability_superspeed_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());
	return true;
}

constexpr uint8_t Device_capability_containerid::bLength;
constexpr uint8_t Device_capability_containerid::bDevCapabilityType;

bool Device_capability_containerid::serialize(Device_capability_containerid_array* const out_array) const
{
	(*out_array)[0] = bLength;
	(*out_array)[1] = bDescriptorType;
	(*out_array)[2] = bDevCapabilityType;
	(*out_array)[3] = bReserved;
	std::copy_n(ContainerID.data(), ContainerID.size(), out_array->data() + 4);

	return true;
}
bool Device_capability_containerid::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	Device_capability_containerid_array temp;
	if(!serialize(&temp))
	{
		return false;
	}

	out_array->insert(temp.data(), temp.size());
	return true;
}
