#pragma once

class Configuration_descriptor
{
public:

protected:
	static constexpr uint8_t bLength = 9;
	static constexpr uint8_t bDescriptorType = 0x02;
	uint16_t wTotalLength
	uint8_t bNumInterfaces
	uint8_t bConfigurationValue
	uint8_t iConfiguration
	uint8_t bmAttributes
	uint8_t bMaxPower
};
