/**
 * @brief Device_descriptor
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

// #include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include "common_util/Byte_util.hpp"

#include <array>

#include <cstdint>

// class Device_descriptor : public Descriptor_base
class Device_descriptor
{
public:

	typedef std::array<uint8_t, 18> Device_descriptor_array;

	bool serialize(Device_descriptor_array* const out_array) const;
	bool deserialize(const Device_descriptor_array& array);

	static constexpr uint16_t build_bcd(const uint8_t major, const uint8_t minor, const uint8_t subminor)
	{
		//0xJJMN
		return Byte_util::make_u16(major, ((minor & 0x0F) << 4) | (subminor & 0x0F));
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
