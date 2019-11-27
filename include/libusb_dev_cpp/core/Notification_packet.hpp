/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

#include <array>

#include <cstdint>

class Notification_packet
{
public:

	typedef std::array<uint8_t, 8> Notification_packet_array;

	bool serialize(Buffer_adapter_base* const out_array) const;
	bool serialize(Notification_packet_array* const out_array) const;
	bool deserialize(const Notification_packet_array& in_array);

	uint8_t bmRequestType;
	uint8_t bNotification;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
protected:
};