#pragma once

#include <array>

#include <cstdint>

class Device_descriptor
{
public:

	typedef std::array<uint8_t, 18> Device_descriptor_array;

	bool serialize(Device_descriptor_array* const out_array);
	bool deserialize(const Device_descriptor_array& array);

protected:
	static constexpr uint8_t bLength = 18;
	static constexpr uint8_t bDescriptorType = 0x01;
	std::array<uint8_t, 2> bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize;
	uint16_t idVendor;
	uint16_t idProduct;
	std::array<uint8_t, 2> bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
};
