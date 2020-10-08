/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2020 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"
#include "libusb_dev_cpp/core/usb_common.hpp"

#include <memory>

// USB 3.2 Revision 1.0 Table 9-12
class BOS_store : public Descriptor_base
{
public:
	typedef std::shared_ptr<BOS_store> BOS_store_ptr;
	typedef std::shared_ptr<const BOS_store> BOS_store_const_ptr;

	BOS_store()
	{

	}
	bool serialize(Buffer_adapter_tx* const out_array) const override;

	size_t size() const override
	{
		return bLength;
	}
	size_t total_size() const;

	static constexpr uint8_t bLength = 5;
	static constexpr uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::BOS);
	// uint16_t wTotalLength; // calculated at runtime
	uint8_t bNumDeviceCaps;

	//a list of Device capability descriptors
	Intrusive_list device_cap_list;
};
