/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#pragma once

#include "libusb_dev_cpp/usb_common.hpp"

#include "libusb_dev_cpp/descriptor/Device_descriptor.hpp"

#include "libusb_dev_cpp/util/Config_desc_table.hpp"
#include "libusb_dev_cpp/util/Iface_desc_table.hpp"
#include "libusb_dev_cpp/util/String_desc_table.hpp"
#include "libusb_dev_cpp/util/Endpoint_desc_table.hpp"

//TODO: for class data, a linked list of base class things might actually work well. CDC needs to send a lot of data after the iface before the ep

class Descriptor_table
{
public:

	typedef std::shared_ptr<Device_descriptor> Device_desc_ptr;
	typedef std::shared_ptr<const Device_descriptor> Device_desc_const_ptr;
	
	bool set_device_descriptor(const Device_descriptor& desc, const uint8_t idx)
	{
		if(idx != 0)
		{
			return false;
		}

		m_dev_desc = std::make_shared<Device_descriptor>(desc);
		return true;
	}

	Device_desc_const_ptr get_device_descriptor(const uint8_t idx) const
	{
		return m_dev_desc;
	}

	// void set_config_descriptor(const Configuration_descriptor& desc, const uint8_t idx)
	// {
	// 	m_config_table.set_config(idx, desc);
	// }

	void set_config_descriptor(const Config_desc_table::Config_desc_ptr& desc, const uint8_t idx)
	{
		m_config_table.set_config(idx, desc);
	}

	Config_desc_table::Config_desc_const_ptr get_config_descriptor(const uint8_t idx) const
	{
		return m_config_table.get_config(idx);
	}

	Config_desc_table::Config_desc_ptr get_config_descriptor(const uint8_t idx)
	{
		return m_config_table.get_config(idx);
	}

	void set_interface_descriptor(const Interface_descriptor& desc, const uint8_t idx)
	{
		m_iface_table.set_config(idx, desc);
	}

	Iface_desc_table::Iface_desc_ptr get_interface_descriptor(const uint8_t idx)
	{
		return m_iface_table.get_config(idx);
	}

	Iface_desc_table::Iface_desc_const_ptr get_interface_descriptor(const uint8_t idx) const
	{
		return m_iface_table.get_config(idx);
	}

	void set_endpoint_descriptor(const Endpoint_descriptor& desc, const uint8_t idx)
	{
		m_endpoint_table.set_config(idx, desc);
	}

	Endpoint_desc_table::Endpoint_desc_ptr get_endpoint_descriptor(const uint8_t idx)
	{
		return m_endpoint_table.get_config(idx);
	}

	Endpoint_desc_table::Endpoint_desc_const_ptr get_endpoint_descriptor(const uint8_t idx) const
	{
		return m_endpoint_table.get_config(idx);
	}

	void set_string_descriptor(const String_descriptor_base& desc, const String_descriptor_zero::LANGID lang, const uint8_t idx)
	{
		String_desc_table* const string_table = m_string_table.get_table(lang);
		string_table->set_config(idx, desc);
	}
	void set_string_descriptor(const String_desc_table::String_desc_ptr& desc, const String_descriptor_zero::LANGID lang, const uint8_t idx)
	{
		String_desc_table* const string_table = m_string_table.get_table(lang);
		string_table->set_config(idx, desc);
	}

	String_desc_table::String_desc_ptr get_string_descriptor(const String_descriptor_zero::LANGID lang, const uint8_t idx)
	{
		String_desc_table* const string_table = m_string_table.get_table(lang);
		if(!string_table)
		{
			return String_desc_table::String_desc_ptr();
		}

		return string_table->get_config(idx);
	}

	String_desc_table::String_desc_const_ptr get_string_descriptor(const String_descriptor_zero::LANGID lang, const uint8_t idx) const
	{
		const String_desc_table* string_table = m_string_table.get_table(lang);
		if(!string_table)
		{
			return String_desc_table::String_desc_const_ptr();
		}
		
		return string_table->get_config(idx);
	}
#if 0
	bool set_descriptor(const Desc_base_ptr& desc, const USB_common::DESCRIPTOR_TYPE type, const uint8_t idx)
	{
		bool ret = false;
		switch(type)
		{
			case USB_common::DESCRIPTOR_TYPE::DEVICE:
			{
				auto dev_ptr = std::dynamic_pointer_cast<Device_descriptor>(desc);
				if(dev_ptr && (idx == 0))
				{
					m_dev_desc = dev_ptr;
					ret = true;
				}
				else
				{
					ret = false;
				}
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::CONFIGURATION:
			{
				auto conf_ptr = std::dynamic_pointer_cast<Configuration_descriptor>(desc);
				if(conf_ptr)
				{
					m_config_table.set_config(idx, conf_ptr);
					ret = true;
				}
				else
				{
					ret = false;
				}
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::STRING:
			{
				//out_desc = m_string_table.get_config(idx);
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::INTERFACE:
			{
				auto iface_ptr = std::dynamic_pointer_cast<Interface_descriptor>(desc);
				if(iface_ptr)
				{
					m_iface_table.set_config(idx, iface_ptr);
					ret = true;
				}
				else
				{
					ret = false;
				}
				break;
			}
			case USB_common::DESCRIPTOR_TYPE::ENDPOINT:
			{
				auto ep_ptr = std::dynamic_pointer_cast<Endpoint_descriptor>(desc);
				if(ep_ptr)
				{
					m_endpoint_table.set_config(idx, ep_ptr);
					ret = true;
				}
				else
				{
					ret = false;
				}
				break;
			}
			default:
			{
				break;
			}
		}
		return ret;
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
#endif
protected:
	
	Device_desc_ptr m_dev_desc;

	Config_desc_table m_config_table;
	Endpoint_desc_table m_endpoint_table;
	Iface_desc_table m_iface_table;
	Multilang_string_desc_table m_string_table;
};