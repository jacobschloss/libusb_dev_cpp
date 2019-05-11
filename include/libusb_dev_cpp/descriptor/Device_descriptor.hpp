#pragma once

class Device_descriptor
{
public:

protected:
	uint8_t bLength;
	uint8_t bDescriptorType;
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
