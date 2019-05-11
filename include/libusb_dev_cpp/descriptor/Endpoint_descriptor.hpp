#pragma once

#include <cstdint>

class Endpoint_descriptor
{
public:

protected:
	static constexpr uint8_t bLength = 7;
	static constexpr uint8_t bDescriptorType = 0x05;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};
