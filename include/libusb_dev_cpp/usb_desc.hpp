#pragma once

#include <array>

class USB_descriptor_base
{
public:

	virtual bool serialize();
	virtual bool deserialize();

protected:
};









