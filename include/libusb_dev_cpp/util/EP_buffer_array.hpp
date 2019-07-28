/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

#include <array>

template <size_t LEN, size_t ALLIGN>
class EP_buffer_array : public Buffer_adapter_base
{
public:

	EP_buffer_array() : Buffer_adapter_base(m_buf.data(), m_buf.size())
	{

	}

protected:
	alignas(ALLIGN) std::array<uint8_t, LEN> m_buf;
};