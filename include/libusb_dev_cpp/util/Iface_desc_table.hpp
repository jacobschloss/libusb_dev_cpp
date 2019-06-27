/**
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

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