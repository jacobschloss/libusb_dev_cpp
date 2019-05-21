#include "libusb_dev_cpp/usb_core.hpp"

bool USB_core::initialize(usb_driver_base* driver, const uint8_t ep0size)
{
	return false;
}

bool USB_core::poll()
{
	return false;
}

bool USB_core::enable()
{
	return m_driver->enable();	
}
bool USB_core::disable()
{
	return m_driver->disable();	
}

bool USB_core::connect()
{
	return m_driver->connect();
}
bool USB_core::disconnect()
{
	return m_driver->disconnect();
}