/**
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

#include <algorithm>

size_t Buffer_adapter_base::insert(const uint8_t* buf_ptr, const size_t len)
{
	const size_t num_to_copy = std::min(len, capacity());
	std::copy_n(buf_ptr, num_to_copy, m_buf_ptr);

	return num_to_copy;
}