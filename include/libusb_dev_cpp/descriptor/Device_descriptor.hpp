/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include "common_util/Byte_util.hpp"

#include <array>

#include <cstdint>

class Device_descriptor : public Descriptor_base
{
public:

	typedef std::array<uint8_t, 18> Device_descriptor_array;

	bool serialize(Device_descriptor_array* const out_array) const;
	bool serialize(Buffer_adapter* const out_array) const override;
	bool deserialize(const Device_descriptor_array& array);

	size_t size() const override
	{
		return bLength;
	}

	static constexpr uint8_t bLength = 18;
	static constexpr uint8_t bDescriptorType = 0x01;
	uint16_t bcdUSB;
	uint8_t  bDeviceClass;
	uint8_t  bDeviceSubClass;
	uint8_t  bDeviceProtocol;
	uint8_t  bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t  iManufacturer;
	uint8_t  iProduct;
	uint8_t  iSerialNumber;
	uint8_t  bNumConfigurations;
protected:
};
