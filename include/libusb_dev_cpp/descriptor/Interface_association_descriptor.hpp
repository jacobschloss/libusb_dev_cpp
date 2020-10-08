/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2020 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"
#include "libusb_dev_cpp/core/usb_common.hpp"

#include <array>

#include <cstdint>

// USB 3.2 Revision 1.0 Table 9-23
class Interface_association_descriptor : public Descriptor_base
{
public:

	typedef std::array<uint8_t, 8> Interface_association_descriptor_array;

	bool serialize(Interface_association_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter_tx* const out_array) const override;

	size_t size() const override
	{
		return bLength;
	}

	static constexpr uint8_t bLength = 8;
	static constexpr uint8_t bDescriptorType = static_cast<uint8_t>(USB_common::DESCRIPTOR_TYPE::INTERFACE_ASSOCIATION);
	uint8_t bFirstInterface;
	uint8_t bInterfaceCount;
	uint8_t bFunctionClass;
	uint8_t bFunctionSubClass;
	uint8_t bFunctionProtocol;
	uint8_t iFunction;
protected:
};
