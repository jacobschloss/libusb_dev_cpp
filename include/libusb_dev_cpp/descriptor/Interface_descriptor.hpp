#pragma once

#include <cstdint>

class Interface_descriptor
{
public:

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
