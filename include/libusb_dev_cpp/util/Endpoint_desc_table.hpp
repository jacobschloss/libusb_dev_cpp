#pragma once

#include "libusb_dev_cpp/descriptor/Endpoint_descriptor.hpp"

#include "libusb_dev_cpp/util/Desc_table_base.hpp"

#include <string>

class Endpoint_desc_table : public Desc_table_base<Endpoint_descriptor>
{
public:

	using Endpoint_desc_ptr = Desc_ptr;
	using Endpoint_desc_const_ptr = Desc_const_ptr;

private:

};