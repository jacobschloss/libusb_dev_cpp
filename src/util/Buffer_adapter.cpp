#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

#include <algorithm>

size_t Buffer_adapter::insert(const uint8_t buf)
{
	return insert(&buf, 1);
}

size_t Buffer_adapter::insert(const uint8_t* buf, const size_t len)
{
	const size_t num_to_copy = std::min(len, capacity());

	std::copy_n(buf, num_to_copy, buf_ptr + rem_len);

	rem_len += num_to_copy;

	return num_to_copy;
}
