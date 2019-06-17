#pragma once

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/descriptor/Device_descriptor.hpp"

#include "libusb_dev_cpp/util/Config_desc_table.hpp"
#include "libusb_dev_cpp/util/Iface_desc_table.hpp"
#include "libusb_dev_cpp/util/String_desc_table.hpp"
#include "libusb_dev_cpp/util/Endpoint_desc_table.hpp"

class Descriptor_table
{
public:
	
	void set_descriptor(const Desc_base_ptr& desc, const USB_common::DESCRIPTOR_TYPE type, const uint8_t idx)
	{
		switch(type)
		{
			case USB_common::DESCRIPTOR_TYPE::DEVICE:
			{
				if(idx == 0)
				{
					m_dev_desc = std::dynamic_pointer_cast<Device_descriptor>(desc);
				}
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::CONFIGURATION:
			{
				m_config_table.set_config(idx, std::dynamic_pointer_cast<Configuration_descriptor>(desc));
				// m_config_table.set_config(idx, *desc);
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::STRING:
			{
				//out_desc = m_string_table.get_config(idx);
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::INTERFACE:
			{
				// out_desc = m_iface_table.get_config(idx);
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::ENDPOINT:
			{
				// out_desc = m_endpoint_table.get_config(idx);
				break;
			}
			default:
			{
				break;
			}
		}
	}

	Desc_base_const_ptr get_descriptor(const USB_common::DESCRIPTOR_TYPE type, const uint8_t idx)
	{
		Desc_base_const_ptr out_desc;

		switch(type)
		{
			case USB_common::DESCRIPTOR_TYPE::DEVICE:
			{
				if(idx == 0)
				{
					out_desc = m_dev_desc;
				}
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::CONFIGURATION:
			{
				out_desc = m_config_table.get_config(idx);
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::STRING:
			{
				//out_desc = m_string_table.get_config(idx);
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::INTERFACE:
			{
				out_desc = m_iface_table.get_config(idx);
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::ENDPOINT:
			{
				out_desc = m_endpoint_table.get_config(idx);
				break;
			}
			default:
			{
				break;
			}
		}
		return out_desc;
	}

protected:
	
	typedef std::shared_ptr<Device_descriptor> Device_desc_ptr;
	typedef std::shared_ptr<const Device_descriptor> Device_desc_const_ptr;

	Device_desc_ptr m_dev_desc;

	Config_desc_table m_config_table;
	Endpoint_desc_table m_endpoint_table;
	Iface_desc_table m_iface_table;
	String_desc_table m_string_table;
};