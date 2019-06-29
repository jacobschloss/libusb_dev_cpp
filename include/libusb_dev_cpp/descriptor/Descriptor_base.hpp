/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

#include "freertos_cpp_util/util/Intrusive_list.hpp"

#include <memory>

class Descriptor_base : public Intrusive_list_node
{
public:
	virtual ~Descriptor_base()
	{

	}

	virtual bool serialize(Buffer_adapter* const out_array) const = 0;

	virtual size_t size() const = 0;
};

typedef std::shared_ptr<Descriptor_base> Desc_base_ptr;
typedef std::shared_ptr<const Descriptor_base> Desc_base_const_ptr;
