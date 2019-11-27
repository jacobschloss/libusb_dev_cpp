/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/core/Notification_packet.hpp"

#include "common_util/Byte_util.hpp"

bool Notification_packet::serialize(Notification_packet_array* const out_array) const
{
	(*out_array)[0] = bmRequestType;

	(*out_array)[1] = bNotification;

	(*out_array)[2] = Byte_util::get_b0(wValue);
	(*out_array)[3] = Byte_util::get_b1(wValue);

	(*out_array)[4] = Byte_util::get_b0(wIndex);
	(*out_array)[5] = Byte_util::get_b1(wIndex);

	(*out_array)[6] = Byte_util::get_b0(wLength);
	(*out_array)[7] = Byte_util::get_b1(wLength);

	return true;
}
bool Notification_packet::deserialize(const Notification_packet_array& in_array)
{
	bmRequestType = in_array[0];

	bNotification = in_array[1];

	wValue = Byte_util::make_u16(in_array[3], in_array[2]);
	wIndex = Byte_util::make_u16(in_array[5], in_array[4]);
	wLength = Byte_util::make_u16(in_array[7], in_array[6]);

	return true;
}

bool Notification_packet::serialize(Buffer_adapter_base* const out_array) const
{
	size_t len = 0;

	len += out_array->insert(bmRequestType);

	len += out_array->insert(bNotification);

	len += out_array->insert(Byte_util::get_b0(wValue));
	len += out_array->insert(Byte_util::get_b1(wValue));

	len += out_array->insert(Byte_util::get_b0(wIndex));
	len += out_array->insert(Byte_util::get_b1(wIndex));

	len += out_array->insert(Byte_util::get_b0(wLength));
	len += out_array->insert(Byte_util::get_b1(wLength));

	return len == std::tuple_size< Notification_packet_array >::value;
}