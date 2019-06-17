#pragma once

#include "libusb_dev_cpp/descriptor/Configuration_descriptor.hpp"

#include "libusb_dev_cpp/util/Desc_table_base.hpp"

class Config_desc_table : public Desc_table_base<Configuration_descriptor>
{
public:

	using Config_desc_ptr = Desc_ptr;
	using Config_desc_const_ptr = Desc_const_ptr;

private:

};