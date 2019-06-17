/**
 * @brief Interface_descriptor
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include <array>

#include <cstdint>

class Interface_descriptor : public Descriptor_base
{
public:

	typedef std::array<uint8_t, 9> Interface_descriptor_array;

	bool serialize(Interface_descriptor_array* const out_array);
	bool deserialize(const Interface_descriptor_array& array);

protected:
	static constexpr uint8_t bLength = 9;
	static constexpr uint8_t bDescriptorType = 0x04;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
};
