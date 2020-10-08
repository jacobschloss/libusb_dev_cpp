/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2020 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/descriptor/Binary_object_store.hpp"

#include "common_util/Byte_util.hpp"

constexpr uint8_t BOS_store::bLength;
constexpr uint8_t BOS_store::bDescriptorType;

bool BOS_store::serialize(Buffer_adapter_tx* const out_array) const
{
	if(out_array->capacity() < size())
	{
		return false;
	}

	//walk the list can calculate the total size
	const size_t total_len = total_size();
	if(total_len > 65535)
	{
		return false;
	}
	const uint16_t wTotalLength = total_len;

	out_array->insert(bLength);
	out_array->insert(bDescriptorType);
	out_array->insert(Byte_util::get_b0(wTotalLength));
	out_array->insert(Byte_util::get_b1(wTotalLength));
	out_array->insert(bNumDeviceCaps);

	//add as many caps as can fit
	bool ret = true;
	if(!device_cap_list.empty())
	{
		Descriptor_base const * node = device_cap_list.front<Descriptor_base>();
		do
		{
			if(!node->serialize(out_array))
			{
				ret = false;
				break;
			}
			node = node->next<Descriptor_base>();
		} while(node != nullptr);
	}

	return ret;
}
size_t BOS_store::total_size() const
{
	size_t len = bLength;
	if(!device_cap_list.empty())
	{
		Descriptor_base const * node = device_cap_list.front<Descriptor_base>();
		do
		{
			len += node->size();
			node = node->next<Descriptor_base>();
		} while(node != nullptr);
	}

	return len;
}
