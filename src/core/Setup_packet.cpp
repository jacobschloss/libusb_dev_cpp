#include "libusb_dev_cpp/core/Setup_packet.hpp"

#include "common_util/Byte_util.hpp"

bool Setup_packet::serialize(Setup_packet_array* const out_array)
{
	(*out_array)[0] = bmRequestType;

	(*out_array)[1] = bRequest;

	(*out_array)[2] = Byte_util::get_b0(wValue);
	(*out_array)[3] = Byte_util::get_b1(wValue);

	(*out_array)[4] = Byte_util::get_b0(wIndex);
	(*out_array)[5] = Byte_util::get_b1(wIndex);

	(*out_array)[6] = Byte_util::get_b0(wLength);
	(*out_array)[7] = Byte_util::get_b1(wLength);

	return true;
}
bool Setup_packet::deserialize(const Setup_packet_array& in_array)
{
	bmRequestType = in_array[0];

	bRequest = in_array[1];

	wValue = Byte_util::make_u16(in_array[3], in_array[2]);
	wIndex = Byte_util::make_u16(in_array[5], in_array[4]);
	wLength = Byte_util::make_u16(in_array[7], in_array[6]);

	return true;
}

bool Setup_packet::get_request_type(Request_type* const request_type)
{
	return request_type->deserialize(bmRequestType);
}