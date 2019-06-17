#pragma once

#include "libusb_dev_cpp/descriptor/Interface_descriptor.hpp"

#include "libusb_dev_cpp/util/Desc_table_base.hpp"

class Iface_desc_table : public Desc_table_base<Interface_descriptor>
{
public:

	using Iface_desc_ptr = Desc_ptr;
	using Iface_desc_const_ptr = Desc_const_ptr;

private:

};