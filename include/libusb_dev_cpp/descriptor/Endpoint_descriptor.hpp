/**
 * @brief Endpoint_descriptor
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/descriptor/Descriptor_base.hpp"

#include <array>

#include <cstdint>

class Endpoint_descriptor : public Descriptor_base
{
public:

	typedef std::array<uint8_t, 7> Endpoint_descriptor_array;

	bool serialize(Endpoint_descriptor_array* const out_array);
	bool deserialize(const Endpoint_descriptor_array& array);

protected:
	static constexpr uint8_t bLength = 7;
	static constexpr uint8_t bDescriptorType = 0x05;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};
