/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/core/Notification_packet.hpp"

#include "libusb_dev_cpp/class/cdc/cdc.hpp"

#include "libusb_dev_cpp/util/Buffer_adapter.hpp"

#include "common_util/Byte_util.hpp"

#include <cstdint>

class NETWORK_CONNECTION_NOTIFICATION
{
public:

	NETWORK_CONNECTION_NOTIFICATION()
	{	
		m_notify_packet.bmRequestType = 0xA1;
		m_notify_packet.bNotification = static_cast<uint8_t>(CDC::CDC_NOTIFICATION::NETWORK_CONNECTION);
		m_notify_packet.wValue = 0;
		m_notify_packet.wIndex = 0;
		m_notify_packet.wLength = 0;
	}

	bool is_connected() const
	{
		return m_notify_packet.wValue == 1;
	}

	void set_connection(const bool set)
	{
		if(set)
		{
			m_notify_packet.wValue = 1;
		}
		else
		{
			m_notify_packet.wValue = 0;
		}
	}

	Notification_packet m_notify_packet;

	bool serialize(Buffer_adapter_base* const out_array) const
	{
		out_array->reset();

		return m_notify_packet.serialize(out_array);
	}

};

class SERIAL_STATE_NOTIFICATION
{
public:

	SERIAL_STATE_NOTIFICATION()
	{	
		m_notify_packet.bmRequestType = 0xA1;
		m_notify_packet.bNotification = static_cast<uint8_t>(CDC::CDC_NOTIFICATION::SERIAL_STATE);
		m_notify_packet.wValue = 0;
		m_notify_packet.wIndex = 0;
		m_notify_packet.wLength = 2;
	}

	Notification_packet m_notify_packet;

	uint16_t bmUartState;
	typedef std::array<uint8_t, 2> Uart_state_array;

	bool bOverRun() const
	{
		return bmUartState & Byte_util::bv_16(6);
	}
	bool bParity() const
	{
		return bmUartState & Byte_util::bv_16(5);
	}
	bool bFraming() const
	{
		return bmUartState & Byte_util::bv_16(4);
	}
	bool bRingSignal() const
	{
		return bmUartState & Byte_util::bv_16(3);
	}
	bool bBreak() const
	{
		return bmUartState & Byte_util::bv_16(2);
	}
	bool bTxCarrier() const
	{
		return bmUartState & Byte_util::bv_16(1);
	}
	bool bRxCarrier() const
	{
		return bmUartState & Byte_util::bv_16(0);
	}

	void set_bOverRun(const bool set)
	{
		bmUartState = Byte_util::set_bv_16(bmUartState, set, 6);
	}
	void set_bParity(const bool set)
	{
		bmUartState = Byte_util::set_bv_16(bmUartState, set, 5);
	}
	void set_bFraming(const bool set)
	{
		bmUartState = Byte_util::set_bv_16(bmUartState, set, 4);
	}
	void set_bRingSignal(const bool set)
	{
		bmUartState = Byte_util::set_bv_16(bmUartState, set, 3);
	}
	void set_bBreak(const bool set)
	{
		bmUartState = Byte_util::set_bv_16(bmUartState, set, 2);
	}
	void set_bTxCarrier(const bool set)
	{
		bmUartState = Byte_util::set_bv_16(bmUartState, set, 1);
	}
	void set_bRxCarrier(const bool set)
	{
		bmUartState = Byte_util::set_bv_16(bmUartState, set, 0);
	}

	bool serialize(Buffer_adapter_base* const out_array) const
	{
		out_array->reset();

		if(!m_notify_packet.serialize(out_array))
		{
			return false;
		}

		out_array->insert(Byte_util::get_b0(bmUartState));
		out_array->insert(Byte_util::get_b1(bmUartState));

		return true;
	}

	bool deserialize(const Buffer_adapter_base* buf)
	{
		if(buf->size() != 2)
		{
			return false;
		}

		bmUartState = Byte_util::make_u16(buf->data()[1], buf->data()[0]);

		return true;
	}
};

class CONNECTION_SPEED_CHANGE_NOTIFICATION
{
public:
	CONNECTION_SPEED_CHANGE_NOTIFICATION()
	{	
		m_notify_packet.bmRequestType = 0xA1;
		m_notify_packet.bNotification = static_cast<uint8_t>(CDC::CDC_NOTIFICATION::CONNECTION_SPEED_CHANGE);
		m_notify_packet.wValue = 0;
		m_notify_packet.wIndex = 0;
		m_notify_packet.wLength = 8;
	}

	Notification_packet m_notify_packet;

	uint32_t DLBitRate;
	uint32_t ULBitRate;

	bool serialize(Buffer_adapter_base* const out_array) const
	{
		out_array->reset();

		if(!m_notify_packet.serialize(out_array))
		{
			return false;
		}

		out_array->insert(Byte_util::get_b0(DLBitRate));
		out_array->insert(Byte_util::get_b1(DLBitRate));
		out_array->insert(Byte_util::get_b2(DLBitRate));
		out_array->insert(Byte_util::get_b3(DLBitRate));

		out_array->insert(Byte_util::get_b0(ULBitRate));
		out_array->insert(Byte_util::get_b1(ULBitRate));
		out_array->insert(Byte_util::get_b2(ULBitRate));
		out_array->insert(Byte_util::get_b3(ULBitRate));

		return true;
	}

	bool deserialize(const Buffer_adapter_base* buf)
	{
		if(buf->size() != 2)
		{
			return false;
		}

		DLBitRate = Byte_util::make_u32(buf->data()[3], buf->data()[2], buf->data()[1], buf->data()[0]);
		ULBitRate = Byte_util::make_u32(buf->data()[7], buf->data()[6], buf->data()[5], buf->data()[4]);

		return true;
	}
};
