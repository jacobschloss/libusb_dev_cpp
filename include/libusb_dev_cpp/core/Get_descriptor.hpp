/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include <cstdint>

class Get_descriptor
{
public:

	enum class DESCRIPTOR_TYPES : uint8_t
	{
		DEVICE                    = 0x01,
		CONFIGURATION             = 0x02,
		STRING                    = 0x03,
		INTERFACE                 = 0x04,
		ENDPOINT                  = 0x05,
		DEVICE_QUALIFIER          = 0x06,
		OTHER_SPEED_CONFIGURATION = 0x07,
		INTERFACE_POWER           = 0x08
	};

	bool serialize(uint16_t* const out_wIndex);
	bool deserialize(const uint16_t wIndex);

	DESCRIPTOR_TYPES type;
	uint8_t idx;
};
