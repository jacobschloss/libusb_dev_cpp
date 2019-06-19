/**
 * @brief Endpoint_descriptor
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "freertos_cpp_util/util/Intrusive_list.hpp"

#include <array>

#include <cstdint>

class Endpoint_descriptor : public Intrusive_list_node
{
public:

	enum class ATTRIBUTE_TRANSFER
	{
		CONTROL     = 0x00,
		ISOCHRONOUS = 0x01,
		BULK        = 0x02,
		INTERRUPT   = 0x03
	};

	enum class ATTRIBUTE_SYNCHRONIZATION_TYPE
	{
		NONE         = 0x00,
		ASYNCHRONOUS = 0x01,
		ADAPTIVE     = 0x02,
		SYNCHRONOUS  = 0x03
	};

	enum class ATTRIBUTE_USAGE_TYPE
	{
		DATA              = 0x00,
		FEEDBACK          = 0x01,
		EXPLICIT_FEEDBACK = 0x02
	};

	typedef std::array<uint8_t, 7> Endpoint_descriptor_array;

	bool serialize(Endpoint_descriptor_array* const out_array) const;
	bool deserialize(const Endpoint_descriptor_array& array);

	static constexpr uint8_t bLength = 7;
	static constexpr uint8_t bDescriptorType = 0x05;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};
