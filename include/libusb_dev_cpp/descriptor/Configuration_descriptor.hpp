/**
 * @brief Configuration_descriptor
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include <array>

#include <cstdint>

class Configuration_descriptor
{
public:

	typedef std::array<uint8_t, 18> Configuration_descriptor_array;

	bool serialize(Configuration_descriptor_array* const out_array);
	bool deserialize(const Configuration_descriptor_array& array);

protected:
	static constexpr uint8_t bLength = 9;
	static constexpr uint8_t bDescriptorType = 0x02;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
};
