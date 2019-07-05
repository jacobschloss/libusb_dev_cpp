/**
 * @author Jacob Schloss <jacob.schloss@suburbanembedded.com>
 * @copyright Copyright (c) 2019 Suburban Embedded. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/driver/stm32/stm32_h7xx_otghs.hpp"

#include "libusb_dev_cpp/driver/cpu/Cortex_m7.hpp"

#include "STM32H7xx/Include/stm32h7xx.h"

#include "common_util/Byte_util.hpp"
#include "common_util/Register_util.hpp"

#include "uart1_printf.hpp"

#include <vector>

namespace
{
	static volatile USB_OTG_GlobalTypeDef* const OTG  = reinterpret_cast<volatile USB_OTG_GlobalTypeDef*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
	static volatile USB_OTG_DeviceTypeDef* const OTGD = reinterpret_cast<volatile USB_OTG_DeviceTypeDef*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
	static volatile uint32_t* const OTGPCTL           = reinterpret_cast<volatile uint32_t*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE);

	static inline volatile uint32_t* get_ep_fifo(const uint8_t ep)
	{
		return reinterpret_cast<volatile uint32_t*>(USB1_OTG_HS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep * USB_OTG_FIFO_SIZE));
	}
	static inline volatile USB_OTG_INEndpointTypeDef* get_ep_in(const uint8_t ep)
	{
		return reinterpret_cast<volatile USB_OTG_INEndpointTypeDef*>(USB1_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep * USB_OTG_EP_REG_SIZE));
	}
	static inline volatile USB_OTG_OUTEndpointTypeDef* get_ep_out(const uint8_t ep)
	{
		return reinterpret_cast<volatile USB_OTG_OUTEndpointTypeDef*>(USB1_OTG_HS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep * USB_OTG_EP_REG_SIZE));
	}
}

void stm32_h7xx_otghs::core_reset()
{
	//wait for USB to be idle
	Register_util::wait_until_set(&OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL);

	//soft reset
	Register_util::set_bits(&OTG->GRSTCTL,         USB_OTG_GRSTCTL_CSRST);
	Register_util::wait_until_clear(&OTG->GRSTCTL, USB_OTG_GRSTCTL_CSRST);

	//wait for USB to be idle
	Register_util::wait_until_set(&OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL);
}

//you must configure the ep tx fifo in order, one after another
//eg 0, 1, 2, 3
bool stm32_h7xx_otghs::config_ep_tx_fifo(const uint8_t ep, const size_t len)
{
	if(ep != 0)
	{
		if(ep > MAX_NUM_EP)
		{
			return false;
		}

		if((len < 64) || (len > 2048))
		{
			return false;
		}

		const uint32_t DIEPTXF0_HNPTXFSIZ = OTG->DIEPTXF0_HNPTXFSIZ;
		const uint32_t ep0_fsa = _FLD2VAL(USB_OTG_TX0FSA, DIEPTXF0_HNPTXFSIZ);
		const uint32_t ep0_fd  = _FLD2VAL(USB_OTG_TX0FD , DIEPTXF0_HNPTXFSIZ);
		
		uint32_t fsa = ep0_fsa + ep0_fd;

		for(size_t i = 1; i <= ep; i++)
		{
			if(i == ep)
			{
				//length in 32bit
				const size_t len32 = (len+3U) / 4U;

				//fsa must be 32bit aligned
				if((fsa % 4) != 0)
				{
					fsa += 4 - (fsa % 4);
				}

				if((fsa + len32*4U) > MAX_FIFO_LEN_U8)
				{
					return false;
				}

				OTG->DIEPTXF[i-1] = _VAL2FLD(USB_OTG_DIEPTXF_INEPTXFD, len32) | _VAL2FLD(USB_OTG_DIEPTXF_INEPTXSA, fsa);
			}
			else
			{
				const uint32_t DIEPTXF = OTG->DIEPTXF[i-1];
				const uint32_t i_fsa = _FLD2VAL(USB_OTG_DIEPTXF_INEPTXSA, DIEPTXF);
				const uint32_t i_fd  = _FLD2VAL(USB_OTG_DIEPTXF_INEPTXFD, DIEPTXF);		

				fsa = i_fd + i_fsa;
			}
		}
	}
	else
	{
		if((len < 64) || (len > 1024))
		{
			return false;
		}

		//length in 32bit
		const size_t len32 = (len+3U) / 4U;
		uint32_t fsa = RX_FIFO_SIZE;

		//fsa might need to be 32bit aligned
		if((fsa % 4) != 0)
		{
			fsa += 4 - (fsa % 4);
		}

		if((fsa + len32*4U) > MAX_FIFO_LEN_U8)
		{
			return false;
		}

		OTG->DIEPTXF0_HNPTXFSIZ = _VAL2FLD(USB_OTG_TX0FD, len32) | _VAL2FLD(USB_OTG_TX0FSA, fsa);
	}

	return true;
}

stm32_h7xx_otghs::stm32_h7xx_otghs()
{
	m_tx_buffer = nullptr;
	m_rx_buffer = nullptr;

	m_ep0_cfg.num = 0;
	m_ep0_cfg.size = 0;
	m_ep0_cfg.type = EP_TYPE::UNCONF;

	for(size_t i = 0; i < m_rx_ep_cfg.size(); i++)
	{
		m_rx_ep_cfg[i].num = i;
		m_rx_ep_cfg[i].size = 0;
		m_rx_ep_cfg[i].type = EP_TYPE::UNCONF;
	}
	for(size_t i = 0; i < m_rx_ep_cfg.size(); i++)
	{
		m_tx_ep_cfg[i].num = i;
		m_tx_ep_cfg[i].size = 0;
		m_tx_ep_cfg[i].type = EP_TYPE::UNCONF;
	}
}
stm32_h7xx_otghs::~stm32_h7xx_otghs()
{

}

bool stm32_h7xx_otghs::initialize()
{
	if(!m_rx_buffer)
	{
		uart1_log<64>(LOG_LEVEL::FATAL, "stm32_h7xx_otghs::initialize", "m_rx_buffer is null");
	}

	for(size_t i = 0; i < m_rx_buffer->get_num_ep(); i++)
	{
		Buffer_adapter_base* rx_buf = m_rx_buffer->poll_allocate_buffer(i);
		if(rx_buf == nullptr)
		{
			for(size_t j = 0; j < m_rx_buffer->get_num_ep(); j++)
			{
				Buffer_adapter_base* buf = m_rx_buffer->get_buffer(j);
				if(buf)
				{
					m_rx_buffer->release_buffer(j, buf);
				}
			}

			uart1_log<64>(LOG_LEVEL::FATAL, "stm32_h7xx_otghs::initialize", "could not preallocate buffer for ep %d", i);

			return false;
		}

		m_rx_buffer->set_buffer(i, rx_buf);
	}

	return true;
}

void stm32_h7xx_otghs::get_info()
{

}

bool stm32_h7xx_otghs::enable()
{
	//reset usb if it is on
	if(RCC->AHB1ENR & (RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN))
	{
		if(!disable())
		{
			return false;
		}
	}

	//enable usb core clock and ulpi clock for USB1
	Register_util::set_bits(&RCC->AHB1ENR, RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN);
	//wait for USB to be idle
	Register_util::wait_until_set(&OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL);

	//ULPI
	Register_util::clear_bits(&(OTG->GUSBCFG), USB_OTG_GCCFG_PWRDWN);
	Register_util::clear_bits(&(OTG->GUSBCFG), USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);
	Register_util::clear_bits(&(OTG->GUSBCFG), USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);

	//soft reset
	core_reset();

	//soft disconnect
	Register_util::set_bits(&OTGD->DCTL, USB_OTG_DCTL_SDIS);

	//start clocks, no sleep gate
	Register_util::clear_bits(OTGPCTL, USB_OTG_PCGCR_PHYSUSP | USB_OTG_PCGCR_GATEHCLK | USB_OTG_PCGCR_STPPCLK);

	//config as device
	Register_util::clear_bits(&OTG->GUSBCFG, 
		USB_OTG_GUSBCFG_PTCI       | 
		USB_OTG_GUSBCFG_PCCI       | 
		USB_OTG_GUSBCFG_ULPICSM    | 
		USB_OTG_GUSBCFG_PHYLPCS    | 
		USB_OTG_GUSBCFG_HNPCAP     | 
		USB_OTG_GUSBCFG_SRPCAP
		);
	Register_util::set_bits(&OTG->GUSBCFG, USB_OTG_GUSBCFG_FDMOD);
	Register_util::set_bits(&OTG->GUSBCFG, USB_OTG_GUSBCFG_ULPIAR | USB_OTG_GUSBCFG_ULPIIPD);
	Register_util::mask_set_bits(&OTG->GUSBCFG, USB_OTG_GUSBCFG_TRDT, _VAL2FLD(USB_OTG_GUSBCFG_TRDT,   0x09));
	Register_util::mask_set_bits(&OTG->GUSBCFG, USB_OTG_GUSBCFG_TOCAL, _VAL2FLD(USB_OTG_GUSBCFG_TOCAL, 0x00));
	
	//reset since we picked a phy
	core_reset();

	//power down
	Register_util::clear_bits(&OTG->GCCFG, (1U << 20) | (1U << 19) | (1U << 18) | (1U << 17) | USB_OTG_GCCFG_PWRDWN);
	
	//No vbus sense
	Register_util::clear_bits<uint32_t>(&OTG->GCCFG, 1U << 21);

	//force B state valid
	Register_util::set_bits<uint32_t>(&OTG->GOTGCTL, USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL);

	// Register_util::mask_set_bits(
		// &OTGD->DCFG, 
		// USB_OTG_DCFG_PERSCHIVL | USB_OTG_DCFG_PFIVL | USB_OTG_DCFG_DAD | USB_OTG_DCFG_DSPD,
		// _VAL2FLD(USB_OTG_DCFG_PERSCHIVL, 0x00) | _VAL2FLD(USB_OTG_DCFG_PFIVL, 0x00) | _VAL2FLD(USB_OTG_DCFG_DAD, 0x00) | _VAL2FLD(USB_OTG_DCFG_DSPD, 0x00)
		// );
	Register_util::set_bits(
		&OTGD->DCFG, 
		_VAL2FLD(USB_OTG_DCFG_PERSCHIVL, 0x00) | _VAL2FLD(USB_OTG_DCFG_PFIVL, 0x00) | _VAL2FLD(USB_OTG_DCFG_DAD, 0x00) | _VAL2FLD(USB_OTG_DCFG_DSPD, 0x00)
		);
	// Register_util::set_bits(&OTGD->DCFG, USB_OTG_DCFG_NZLSOHSK);


	//reset fifo assignments
	for (size_t i = 1; i < MAX_NUM_EP; i++)
	{
		OTG->DIEPTXF[i-1] = _VAL2FLD(USB_OTG_DIEPTXF_INEPTXFD, 0x0200) | _VAL2FLD(USB_OTG_DIEPTXF_INEPTXSA, 0x0200+0x0200*i);
	}

	//rx fifo
	OTG->GRXFSIZ = RX_FIFO_SIZE;
	//ep0 tx fifo, TX0FD | TX0FSA
	if(!config_ep_tx_fifo(0, 3*(64 + 8 + 4)))
	{
		return false;
	}

	//flush fifo
	flush_all_tx();
	flush_rx();

	//clear core interrupt
	OTG->GINTMSK = 0U;
	OTG->GINTSTS = 0xFFFFFFFF;

	//tx ep int
	OTGD->DAINTMSK = 0U;
	OTGD->DOEPMSK  = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
	OTGD->DIEPMSK  = USB_OTG_DIEPMSK_TOM   | USB_OTG_DIEPMSK_XFRCM;

	//config core interrupt
	OTG->GINTMSK  = USB_OTG_GINTMSK_USBRST   |
					USB_OTG_GINTMSK_ENUMDNEM |
    				USB_OTG_GINTMSK_USBSUSPM |
					USB_OTG_GINTMSK_ESUSPM   |
    				//USB_OTG_GINTMSK_SOFM   |
					USB_OTG_GINTMSK_WUIM     |
					USB_OTG_GINTMSK_IEPINT   |
					USB_OTG_GINTMSK_RXFLVLM  |
					USB_OTG_GINTMSK_IEPINT   |
					USB_OTG_GINTMSK_OEPINT
					;

	//turn on global interrupt
	Register_util::set_bits(&OTG->GAHBCFG, USB_OTG_GAHBCFG_GINT);

	return true;
}
bool stm32_h7xx_otghs::disable()
{
	if(RCC->AHB1ENR & (RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN))
	{
		//reset USB1
		Register_util::set_bits(&RCC->AHB1RSTR,   RCC_AHB1RSTR_USB1OTGHSRST);
		Register_util::clear_bits(&RCC->AHB1RSTR, RCC_AHB1RSTR_USB1OTGHSRST);

		//gate clocks
		Register_util::clear_bits(&RCC->AHB1ENR, RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN);

		//flush pipeline
		Cortex_m7::data_instruction_sync();
	}

	return true;
}

bool stm32_h7xx_otghs::connect()
{
	Register_util::set_bits(&OTG->GCCFG, USB_OTG_GCCFG_PWRDWN);
	Register_util::clear_bits(&OTGD->DCTL, USB_OTG_DCTL_SDIS);

	return true;
}
bool stm32_h7xx_otghs::disconnect()
{
	Register_util::set_bits(&OTGD->DCTL, USB_OTG_DCTL_SDIS);
	Register_util::clear_bits(&OTG->GCCFG, USB_OTG_GCCFG_PWRDWN);

	//flush
	Cortex_m7::data_instruction_sync();

	return true;
}

bool stm32_h7xx_otghs::set_address(const uint8_t addr)
{
	Register_util::mask_set_bits(&OTGD->DCFG, USB_OTG_DCFG_DAD, _VAL2FLD(USB_OTG_DCFG_DAD, uint32_t(addr)));

	return true;
}

bool stm32_h7xx_otghs::ep_config(const ep_cfg& ep)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep.num);

	if(ep_addr == 0)
	{
		if(ep.type != usb_driver_base::EP_TYPE::CONTROL)
		{
			return false;
		}

		volatile USB_OTG_INEndpointTypeDef*  const ep_in  = get_ep_in(ep_addr);
		volatile USB_OTG_OUTEndpointTypeDef* const ep_out = get_ep_out(ep_addr);

		uint8_t mpsize = 0;
		m_ep0_cfg = ep;
		if(ep.size <= 8)
		{
			m_ep0_cfg.size = 8;
			mpsize = 3;
		}
		else if(ep.size <= 16)
		{
			m_ep0_cfg.size = 16;
			mpsize = 2;
		}
		else if(ep.size <= 32)
		{
			m_ep0_cfg.size = 32;
			mpsize = 1;
		}
		else
		{
			m_ep0_cfg.size = 64;
			mpsize = 0;
		}

		OTGD->DAINTMSK |= 0x00010001;

		ep_in->DIEPCTL = 
			// USB_OTG_DIEPCTL_SD0PID_SEVNFRM            |
			USB_OTG_DIEPCTL_SNAK                      | 
			_VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, ep_addr) | 
			_VAL2FLD(USB_OTG_DIEPCTL_EPTYP, 0x00)     | 
			USB_OTG_DIEPCTL_USBAEP                    |
			// _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, m_ep0_cfg.size);
			_VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, mpsize);
		
		ep_out->DOEPTSIZ = 
			_VAL2FLD(USB_OTG_DOEPTSIZ_STUPCNT, 3) |
			USB_OTG_DOEPTSIZ_PKTCNT               |
			_VAL2FLD(USB_OTG_DOEPTSIZ_XFRSIZ, m_ep0_cfg.size);
		
		ep_out->DOEPCTL = 
			// USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
			USB_OTG_DOEPCTL_EPENA          | 
			USB_OTG_DOEPCTL_CNAK   
			;
	}
	else if(USB_common::is_in_ep(ep.num))
	{
		volatile USB_OTG_INEndpointTypeDef* const ep_in = get_ep_in(ep_addr);

		if(!config_ep_tx_fifo(ep_addr, ep.size))
		{
			return false;
		}

		//enable tx interrupt
		OTGD->DAINTMSK |= _VAL2FLD(USB_OTG_DAINTMSK_IEPM, 0x0001 << ep_addr);

		switch(ep.type)
		{
			case usb_driver_base::EP_TYPE::ISOCHRONUS:
			{
				return false;
				break;
			}
			case usb_driver_base::EP_TYPE::INTERRUPT:
			{
				return false;
				break;
			}
			case usb_driver_base::EP_TYPE::BULK:
			{
				ep_in->DIEPCTL = 
					USB_OTG_DIEPCTL_SD0PID_SEVNFRM            | 
					USB_OTG_DIEPCTL_SNAK                      | 
					_VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, ep_addr) | 
					_VAL2FLD(USB_OTG_DIEPCTL_EPTYP, 0x02)     | 
					USB_OTG_DIEPCTL_USBAEP                    | 
					_VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, ep.size);
				break;
			}
			default:
			{
				return false;
			}
		}
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* const ep_out = get_ep_out(ep_addr);
		switch(ep.type)
		{
			case usb_driver_base::EP_TYPE::ISOCHRONUS:
			{
				return false;
				break;
			}
			case usb_driver_base::EP_TYPE::INTERRUPT:
			{
				return false;
				break;
			}
			case usb_driver_base::EP_TYPE::BULK:
			{
				ep_out->DOEPCTL = 
					USB_OTG_DOEPCTL_EPENA                     |
					USB_OTG_DIEPCTL_SD0PID_SEVNFRM            | 
					USB_OTG_DIEPCTL_CNAK                      | 
					_VAL2FLD(USB_OTG_DOEPCTL_EPTYP, 0x02)     | 
					USB_OTG_DIEPCTL_USBAEP                    | 
					_VAL2FLD(USB_OTG_DOEPCTL_MPSIZ, ep.size);
				break;
			}
			default:
			{
				return false;
			}
		}
	}

	return false;
}
bool stm32_h7xx_otghs::ep_unconfig(const uint8_t ep)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep);

	volatile USB_OTG_INEndpointTypeDef*  const ep_in  = get_ep_in(ep_addr);
	volatile USB_OTG_OUTEndpointTypeDef* const ep_out = get_ep_out(ep_addr);
	
	OTGD->DAINTMSK &= ~(0x00010001 << ep_addr);

	Register_util::clear_bits(&ep_in->DIEPCTL, USB_OTG_DIEPCTL_USBAEP);
	flush_tx(ep_addr);

	if(ep_addr != 0)
	{
		if(ep_in->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
		{
			Register_util::set_bits(&ep_in->DIEPCTL, USB_OTG_DIEPCTL_EPDIS);
		}
	}

	ep_in->DIEPINT = 
		(1U << 13) | 
		(1U << 11) | 
		(1U <<  8) | 
		(1U <<  7) | 
		(1U <<  6) | 
		(1U <<  5) | 
		(1U <<  4) | 
		(1U <<  3) | 
		(1U <<  2) | 
		(1U <<  1) | 
		(1U <<  0);

	if(ep_addr != 0)
	{
		OTG->DIEPTXF[ep_addr-1] = _VAL2FLD(USB_OTG_DIEPTXF_INEPTXFD, 0x0200) | _VAL2FLD(USB_OTG_DIEPTXF_INEPTXSA, 0x0200+0x0200*ep_addr);
	}

	Register_util::clear_bits(&ep_out->DOEPCTL, USB_OTG_DOEPCTL_USBAEP);
	if(ep_out->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
	{
		Register_util::set_bits(&ep_out->DOEPCTL, USB_OTG_DOEPCTL_EPDIS);
	}
	ep_out->DOEPINT = 
		(1U << 14) | 
		(1U << 13) | 
		(1U << 12) | 
		(1U <<  8) | 
		(1U <<  7) | 
		(1U <<  6) | 
		(1U <<  5) | 
		(1U <<  4) | 
		(1U <<  3) | 
		(1U <<  2) | 
		(1U <<  1) | 
		(1U <<  0);

	return true;
}

bool stm32_h7xx_otghs::ep_is_stalled(const uint8_t ep)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep);
	bool is_stalled = false;

	if(USB_common::is_in_ep(ep))
	{
		volatile USB_OTG_INEndpointTypeDef* epin = get_ep_in(ep_addr);

		is_stalled = epin->DIEPCTL & USB_OTG_DIEPCTL_STALL;
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = get_ep_out(ep_addr);

		is_stalled = epout->DOEPCTL | USB_OTG_DOEPCTL_STALL;
	}

	return is_stalled;
}
void stm32_h7xx_otghs::ep_stall(const uint8_t ep)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep);

	if(USB_common::is_in_ep(ep))
	{
		volatile USB_OTG_INEndpointTypeDef* epin = get_ep_in(ep_addr);

		Register_util::set_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_STALL);
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = get_ep_out(ep_addr);

		Register_util::set_bits(&epout->DOEPCTL, USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_STALL);
	}
}
void stm32_h7xx_otghs::ep_unstall(const uint8_t ep)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep);
	ep_cfg cfg;

	//todo handle OOB ep
	if(ep == 0)
	{
		cfg = get_ep0_config();
	}
	else if(USB_common::is_in_ep(ep))
	{
		get_tx_ep_config(ep, &cfg);
	}
	else
	{
		get_rx_ep_config(ep, &cfg);
	}

	if(USB_common::is_in_ep(ep))
	{
		volatile USB_OTG_INEndpointTypeDef* epin = get_ep_in(ep_addr);

		Register_util::clear_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_STALL);

		if((cfg.type == EP_TYPE::BULK) || (cfg.type == EP_TYPE::INTERRUPT))
		{
			Register_util::set_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
		}
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = get_ep_out(ep_addr);

		Register_util::clear_bits(&epout->DOEPCTL, USB_OTG_DOEPCTL_STALL);
		if((cfg.type == EP_TYPE::BULK) || (cfg.type == EP_TYPE::INTERRUPT))
		{
			Register_util::set_bits(&epout->DOEPCTL, USB_OTG_DOEPCTL_SD0PID_SEVNFRM);
		}
	}
}

int stm32_h7xx_otghs::ep_write(const uint8_t ep, const uint8_t* buf, const uint16_t len)
{
	uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs::ep_write", "");

	if(!USB_common::is_in_ep(ep))
	{
		uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs::ep_write", "not an in ep");
		return -1;
	}

	const uint8_t ep_addr = USB_common::get_ep_addr(ep);

	volatile USB_OTG_INEndpointTypeDef* const epin = get_ep_in(ep_addr);

	const size_t len32 = (len + 3) / 4;

	//number of words availible
	const uint32_t DTXFSTS = epin->DTXFSTS;
	const uint32_t INEPTFSAV = _FLD2VAL(USB_OTG_DTXFSTS_INEPTFSAV, DTXFSTS);
	if(INEPTFSAV < len32)
	{
		uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs::ep_write", "wanted %d but only %d avail", len32, INEPTFSAV);
		return -1;
	}

	if(ep_addr == 0)
	{
		uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs::ep_write", "ep0");

		Register_util::mask_set_bits(
							&epin->DIEPTSIZ,
							USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
							_VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, len)
						);
		Register_util::mask_set_bits(
			&epin->DIEPCTL, 
			USB_OTG_DIEPCTL_SD0PID_SEVNFRM, 
			USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
	}
	else
	{

		if(epin->DIEPCTL & USB_OTG_DOEPCTL_EPENA)
		{
			//endpoint already active
			return -1;
		}

		Register_util::mask_set_bits(
							&epin->DIEPTSIZ,
							USB_OTG_DIEPTSIZ_MULCNT | USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
							_VAL2FLD(USB_OTG_DIEPTSIZ_MULCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, len)
						);
		
		Register_util::mask_set_bits(
			&epin->DIEPCTL,
			USB_OTG_DIEPCTL_STALL | USB_OTG_DIEPCTL_SD0PID_SEVNFRM,
			USB_OTG_DOEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	}

	volatile uint32_t* const fifo = get_ep_fifo(ep_addr);
	for(size_t i = 0; i < len; i+=4)
	{
		const size_t u8_left = len - i;

		//copy all 4 if avail, otherwise only the remaining
		if(u8_left >= 4)
		{	
			const uint8_t b0 = buf[i + 0];
			const uint8_t b1 = buf[i + 1];
			const uint8_t b2 = buf[i + 2];
			const uint8_t b3 = buf[i + 3];

			*fifo = Byte_util::make_u32(b3, b2, b1, b0);
		}
		else
		{
			uint8_t b0, b1, b2;

			switch(u8_left)
			{
				case 1:
				{
					b0 = buf[i + 0];
					b1 = 0;
					b2 = 0;
					break;
				}
				case 2:
				{
					b0 = buf[i + 0];
					b1 = buf[i + 1];
					b2 = 0;
					break;
				}
				case 3:
				{
					b0 = buf[i + 0];
					b1 = buf[i + 1];
					b2 = buf[i + 2];
					break;
				}
				case 0:
				default:
				{
					b0 = 0;
					b1 = 0;
					b2 = 0;
					break;
				}
			}

			*fifo = Byte_util::make_u32(0, b2, b1, b0);
		}
	}

	return len;
}
int stm32_h7xx_otghs::ep_read(const uint8_t ep, uint8_t* const buf, const uint16_t max_len)
{
	//no data
	if(!(OTG->GINTSTS & USB_OTG_GINTSTS_RXFLVL))
	{
		return 0;
	}

	for(size_t i = 0; i < max_len; i += 4)
	{
		const uint32_t temp = *get_ep_fifo(0);

		const size_t bleft = max_len - i;
		if(bleft >= 4)
		{
			buf[i+0] = Byte_util::get_b0(temp);
			buf[i+1] = Byte_util::get_b1(temp);
			buf[i+2] = Byte_util::get_b2(temp);
			buf[i+3] = Byte_util::get_b3(temp);
		}
		else
		{
			switch(bleft)
			{
				case 1:
				{
					buf[i+0] = Byte_util::get_b0(temp);
					break;
				}
				case 2:
				{
					buf[i+0] = Byte_util::get_b0(temp);
					buf[i+1] = Byte_util::get_b1(temp);
					break;
				}
				case 3:
				{
					buf[i+0] = Byte_util::get_b0(temp);
					buf[i+1] = Byte_util::get_b1(temp);
					buf[i+2] = Byte_util::get_b2(temp);
					break;
				}
				default:
				{
					break;
				}
			}
		}
	}

	return max_len;
}

uint16_t stm32_h7xx_otghs::get_frame_number()
{
	return _FLD2VAL(USB_OTG_DSTS_FNSOF, OTGD->DSTS);
}
size_t stm32_h7xx_otghs::get_serial_number(uint8_t* const buf, const size_t maxlen)
{
	return 0;
}

USB_common::USB_SPEED stm32_h7xx_otghs::get_speed() const
{
	switch(_FLD2VAL(USB_OTG_DSTS_ENUMSPD, OTGD->DSTS))
	{
		case 0:
		{
			return USB_common::USB_SPEED::HS;
		}
		case 2:
		{
			return USB_common::USB_SPEED::LS;
		}
		case 1:
		case 3:
		default:
		{
			return USB_common::USB_SPEED::FS;
		}
	}
}

#if 1
void stm32_h7xx_otghs::poll(const USB_common::Event_callback& func)
{
	USB_common::USB_EVENTS event = USB_common::USB_EVENTS::NONE;
	uint8_t ep_num = 0;

	const uint32_t GINTSTS = OTG->GINTSTS;

	if(GINTSTS & USB_OTG_GINTSTS_SRQINT)
	{
		OTG->GINTSTS = USB_OTG_GINTSTS_SRQINT;
	}
	else if(GINTSTS & USB_OTG_GINTSTS_WKUINT)
	{
		OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;	
	}
	else if(GINTSTS & USB_OTG_GINTSTS_USBRST)
	{
		uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_USBRST", "");

		OTG->GINTSTS = USB_OTG_GINTSTS_USBRST;

		for(uint8_t i = 0; i <= MAX_NUM_EP; i++)
		{
			ep_unconfig(i);
		}

		flush_rx();
		flush_all_tx();
	}
	else if(GINTSTS & USB_OTG_GINTSTS_ESUSP)
	{
		uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_ESUSP", "");
	
		OTG->GINTSTS = USB_OTG_GINTSTS_ESUSP;

		event = USB_common::USB_EVENTS::EARLY_SUSPEND;
	}
	else if(GINTSTS & USB_OTG_GINTSTS_USBSUSP)
	{
		uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_USBSUSP", "");

		OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;

		event = USB_common::USB_EVENTS::SUSPEND;
	}
	else if(GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
	{
		uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_ENUMDNE", "");

		OTG->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;

		event = USB_common::USB_EVENTS::RESET;
	}
	else if(GINTSTS & USB_OTG_GINTSTS_SOF)
	{
		OTG->GINTSTS = USB_OTG_GINTSTS_SOF;

		event = USB_common::USB_EVENTS::SOF;
	}
	else if(GINTSTS & USB_OTG_GINTSTS_IEPINT)
	{
		const uint32_t IEPINT = _FLD2VAL(USB_OTG_DAINT_IEPINT, OTGD->DAINT);

		if(IEPINT != 0)
		{
			ep_num = __builtin_ctz(IEPINT);
			const uint32_t DIEPINT = get_ep_in(ep_num)->DIEPINT;

			if(DIEPINT & USB_OTG_DIEPINT_XFRC)
			{
				//transfer complete
				get_ep_in(ep_num)->DIEPINT = USB_OTG_DIEPINT_XFRC;
				
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_IEPINT", "DIEPINT[%d] XFRC 0x%08X", ep_num, DIEPINT);
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_IEPINT", "event EP_TX");

				{
					Buffer_adapter_base* const curr_tx_buf = m_tx_buffer->get_buffer(ep_num);
					if(curr_tx_buf)
					{
						m_tx_buffer->release_buffer(ep_num, curr_tx_buf);
					}
				}

				//see if there is a new packet to enqueue
				Buffer_adapter_base* const new_tx_buf = m_tx_buffer->poll_dequeue_buffer(ep_num);
				if(new_tx_buf)
				{
					m_tx_buffer->set_buffer(ep_num, new_tx_buf);
					ep_write(0x80 | ep_num, new_tx_buf->data(), new_tx_buf->size());
				}
				else
				{
					m_tx_buffer->set_buffer(ep_num, nullptr);
					get_ep_in(ep_num)->DIEPCTL |= (USB_OTG_DOEPCTL_SNAK);
				}

				event = USB_common::USB_EVENTS::EP_TX;
				ep_num = ep_num | 0x80;
			}
			else if(DIEPINT & USB_OTG_DIEPINT_NAK)
			{
				//NAK tx or rx
				get_ep_in(ep_num)->DIEPINT = USB_OTG_DIEPINT_NAK;
			}
			else if(DIEPINT & USB_OTG_DIEPINT_PKTDRPSTS)
			{
				//isoc out packet dropped
				get_ep_in(ep_num)->DIEPINT = USB_OTG_DIEPINT_PKTDRPSTS;
			}
			else if(DIEPINT & USB_OTG_DIEPINT_TXFIFOUDRN)
			{
				//tx fifo underrun
				get_ep_in(ep_num)->DIEPINT = USB_OTG_DIEPINT_TXFIFOUDRN;
			}
			else if(DIEPINT & USB_OTG_DIEPINT_INEPNE)
			{
				//IN token received with EP mismatch
				//Data on top of non periodic txfifo is for ep other than for in token
				get_ep_in(ep_num)->DIEPINT = USB_OTG_DIEPINT_INEPNE;
			}
			else if(DIEPINT & USB_OTG_DIEPINT_ITTXFE)
			{
				//IN token received when Tx FIFO is empty
				get_ep_in(ep_num)->DIEPINT = USB_OTG_DIEPINT_ITTXFE;
			}
			else if(DIEPINT & USB_OTG_DIEPINT_TOC)
			{
				//Timeout condition
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_IEPINT", "DIEPINT[%d] TOC 0x%08X", ep_num, DIEPINT);
				get_ep_in(ep_num)->DIEPINT = USB_OTG_DIEPINT_TOC;
			}
			else if(DIEPINT & (1 << 2U))
			{
				//AHB bus error
				get_ep_in(ep_num)->DIEPINT = 1 << 2U;
			}
			else if(DIEPINT & USB_OTG_DIEPINT_EPDISD)
			{
				//Endpoint disabled at app request
				get_ep_in(ep_num)->DIEPINT = USB_OTG_DIEPINT_EPDISD;
			}
			else
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_IEPINT", "%d IEPINT  0x%08X",  ep_num, IEPINT);
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_IEPINT", "%d DIEPINT 0x%08X", ep_num, DIEPINT);
			}

			//USB_OTG_DIEPINT_TOC
			//USB_OTG_DIEPINT_TXFE
			//USB_OTG_DIEPINT_EPDISD
			//USB_OTG_DIEPINT_ITTXFE
			//USB_OTG_DIEPINT_INEPNE
			//USB_OTG_DIEPINT_TXFIFOUDRN
			//USB_OTG_DIEPINT_BNA
			//USB_OTG_DIEPINT_PKTDRPSTS
			//USB_OTG_DIEPINT_BERR
			//USB_OTG_DIEPINT_NAK
		}
	}
	else if(GINTSTS & USB_OTG_GINTSTS_OEPINT)
	{
		const uint32_t OEPINT = _FLD2VAL(USB_OTG_DAINT_OEPINT, OTGD->DAINT);

		if(OEPINT != 0)
		{
			ep_num = __builtin_ctz(OEPINT);
			const uint32_t DOEPINT = get_ep_out(ep_num)->DOEPINT;

			if(DOEPINT & USB_OTG_DOEPINT_XFRC)
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_OEPINT", "DOEPINT[%d] XFRC 0x%08X", ep_num, DOEPINT);

				get_ep_out(ep_num)->DOEPINT = USB_OTG_DOEPINT_XFRC;

				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_DOEPINT_XFRC", "event EP_RX");
				event = USB_common::USB_EVENTS::EP_RX;
			}
			else if(DOEPINT & USB_OTG_DOEPINT_STUP)
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_OEPINT", "DOEPINT[%d] STUP 0x%08X", ep_num, DOEPINT);

				get_ep_out(ep_num)->DOEPINT = USB_OTG_DOEPINT_STUP;

				//SETUP phase done
				//no more back to back setup packets
				//decode the setup packet now

				const uint32_t DOEPTSIZ = get_ep_out(ep_num)->DOEPTSIZ;

				const uint32_t XFRSIZ  = _FLD2VAL(USB_OTG_DOEPTSIZ_XFRSIZ, DOEPTSIZ);
				const uint32_t PKTCNT  = _FLD2VAL(USB_OTG_DOEPTSIZ_PKTCNT, DOEPTSIZ);
				const uint32_t STUPCNT = _FLD2VAL(USB_OTG_DOEPTSIZ_STUPCNT, DOEPTSIZ);

				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_DOEPINT_STUP", "XFRSIZ %08X", XFRSIZ);
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_DOEPINT_STUP", "PKTCNT %08X", PKTCNT);
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_DOEPINT_STUP", "STUPCNT %08X", STUPCNT);

				event = USB_common::USB_EVENTS::CTRL_SETUP_PHASE_DONE;
			}
			else if(DOEPINT & USB_OTG_DOEPINT_OTEPSPR)
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_OEPINT", "DOEPINT[%d] OTEPSPR 0x%08X", ep_num, DOEPINT);

				//Status phase received for control write

				//we are now in status phase, send an ACK or stall for the status phase

				event = USB_common::USB_EVENTS::CTRL_STATUS_PHASE;
			}
			
			//USB_OTG_DOEPINT_EPDISD//Endpoint disabled interrupt
			//USB_OTG_DOEPINT_OTEPDIS//OUT token received when endpoint disabled
			//USB_OTG_DOEPINT_B2BSTUP//Back-to-back SETUP packets received (more than 3)
			//USB_OTG_DOEPINT_NYET//NYET response tx
		}
	}
	else if(GINTSTS & USB_OTG_GINTSTS_RXFLVL)
	{
		uart1_log<64>(LOG_LEVEL::INFO, "GINTSTS", "0x%08X", GINTSTS);

		//pop top fifo entry
		const uint32_t GRXSTSP = OTG->GRXSTSP;

		uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_RXFLVL", "GRXSTSP 0x%08X", GRXSTSP);

		const uint32_t STSPHST = (GRXSTSP & 0x08000000) >> 27;
		const uint32_t FRMNUM  = (GRXSTSP & 0x01E00000) >> 21;
		const uint32_t PKTSTS  = _FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, GRXSTSP);
		const uint32_t DPID    = _FLD2VAL(USB_OTG_GRXSTSP_DPID,   GRXSTSP);
		const uint32_t BCNT    = _FLD2VAL(USB_OTG_GRXSTSP_BCNT,   GRXSTSP);
		const uint32_t EPNUM   = _FLD2VAL(USB_OTG_GRXSTSP_EPNUM,  GRXSTSP);
		ep_num = EPNUM;

		uart1_log<64>(LOG_LEVEL::INFO, "STSPHST", "%" PRIu32, STSPHST);
		uart1_log<64>(LOG_LEVEL::INFO, "FRMNUM",  "%" PRIu32, FRMNUM);
		uart1_log<64>(LOG_LEVEL::INFO, "PKTSTS",  "%" PRIu32, PKTSTS);
		uart1_log<64>(LOG_LEVEL::INFO, "DPID",    "%" PRIu32, DPID);
		uart1_log<64>(LOG_LEVEL::INFO, "BCNT",    "%" PRIu32, BCNT);
		uart1_log<64>(LOG_LEVEL::INFO, "EPNUM",   "%" PRIu32, EPNUM);

		switch(PKTSTS)
		{
			case 2://out rx
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_RXFLVL", "2");

				//wait to process until later in the ep isr
				if(BCNT != 0)
				{
					if(ep_num != 0)
					{
						//get active buffer
						Buffer_adapter_base* curr_buf = m_rx_buffer->get_buffer(ep_num);

						//read data from core's fifo into buffer
						curr_buf->reset();
						curr_buf->resize(BCNT);
						ep_read(ep_num, curr_buf->data(), BCNT);

						//enqueue buffer so the application thread can be notified and read it
						m_rx_buffer->poll_enqueue_buffer(ep_num, curr_buf);

						for(size_t i = 0; i < BCNT; i++)
						{
							uart1_printf<16>("%02X ", curr_buf->data()[i]);
						}

						//try to get a new buffer
						Buffer_adapter_base* new_buf = m_rx_buffer->poll_allocate_buffer(ep_num);
						//update the active buffer
						if(new_buf)
						{
							new_buf->reset();
							m_rx_buffer->set_buffer(ep_num, new_buf);
							get_ep_out(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
						}
						else
						{
							//OUT buffer underrun
							//we will need to cnak and epena when the app frees a buffer
							uart1_log<64>(LOG_LEVEL::ERROR, "USB_OTG_GINTSTS_RXFLVL", "rx buffer allocation fail");
							m_rx_buffer->set_buffer(ep_num, nullptr);
							get_ep_out(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_SNAK);
						}
					}
					else
					{
						m_last_ep0_data_packet.resize(BCNT);
						ep_read(ep_num, m_last_ep0_data_packet.data(), BCNT);	
	
						for(size_t i = 0; i < BCNT; i++)
						{
							uart1_printf<16>("%02X ", m_last_ep0_data_packet.data()[i]);
						}
					}

					uart1_printf<16>("\r\n");
				}

				break;
			}
			case 3://out txfr done
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_RXFLVL", "OUT TXFR DONE");

				//rearm ep, dispatch will occur in USB_OTG_DOEPINT_XFRC

				//get_ep_out(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
				break;
			}
			case 6://setup packet rx
			{
				//wait to process until later in the ep isr
				if(BCNT != 0)
				{
					ep_read(ep_num, m_last_setup_packet.data(), BCNT);

					for(size_t i = 0; i < m_last_setup_packet.size(); i++)
					{
						uart1_printf<16>("%02X ", m_last_setup_packet[i]);
					}
					uart1_printf<16>("\r\n");
				}

				break;
			}
			case 4://setup stage done, data stage started
			{
				uart1_log<64>(LOG_LEVEL::INFO, "USB_OTG_GINTSTS_RXFLVL", "event SETUP_PACKET_RX");

				get_ep_out(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
				break;
			}
			case 1://global nak
			default:
			{
				break;
			}
		}
	}

	func(event, ep_num);
}
#else
void stm32_h7xx_otghs::poll(const USB_common::Event_callback& func)
{
	USB_common::USB_EVENTS event = USB_common::USB_EVENTS::RESET;
	uint8_t ep = 0;


	while(true)
	{
		const uint32_t GINTSTS = OTG->GINTSTS;

		if(GINTSTS & USB_OTG_GINTSTS_USBRST)
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_USBRST");

			OTG->GINTSTS = USB_OTG_GINTSTS_USBRST;

			for(uint8_t i = 0; i <= MAX_NUM_EP; i++)
			{
				ep_unconfig(i);
			}

			flush_rx();

			continue;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_ENUMDNE");

			OTG->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;

			event = USB_common::USB_EVENTS::RESET;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_IEPINT)
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_IEPINT");

			// Could check this instead
			// const uint32_t DAINT = OTGD->DAINT;

			bool ret = false;
			for(size_t i = 0; i <= MAX_NUM_EP; i++)
			{
				volatile USB_OTG_INEndpointTypeDef* epin = get_ep_in(i);
				if(epin->DIEPINT & USB_OTG_DIEPINT_XFRC)
				{
					epin->DIEPINT |= USB_OTG_DIEPINT_XFRC;
					event = USB_common::USB_EVENTS::EP_TX;
					ep = i | 0x80;

					ret = true;
					break;
				}
			}
			if(!ret)
			{
				return;
			}
		}
		else if(GINTSTS & USB_OTG_GINTSTS_RXFLVL)
		{
			const uint32_t GRXSTSR = OTG->GRXSTSR;
			ep = _FLD2VAL(USB_OTG_GRXSTSP_EPNUM, GRXSTSR);

			// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_RXFLVL, 0x%08X, %d", GRXSTSR, ep);

			switch(_FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, GRXSTSR))
			{
				case 0x01:
				{
					//GLOBAL OUT NAK, int
					volatile uint32_t GRXSTSP = OTG->GRXSTSP;
					continue;
				}
				case 0x02:
				{
					// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_RXFLVL PKTSTS IN RX - out data rx");

					//OUT packet received
					event = USB_common::USB_EVENTS::EP_RX;
					break;
				}
				case 0x03:
				{
					// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_RXFLVL PKTSTS  0x03 - out transfer done");

					//OUT transfer completed, int
					Register_util::set_bits(&(get_ep_out(ep)->DOEPCTL), USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
					volatile uint32_t GRXSTSP = OTG->GRXSTSP;
					continue;
				}
				case 0x04:
				{
					// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_RXFLVL PKTSTS 0x04 - setup transaction done");

					//SETUP transaction completed, int
					Register_util::set_bits(&(get_ep_out(ep)->DOEPCTL), USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
					volatile uint32_t GRXSTSP = OTG->GRXSTSP;

					// event = USB_common::USB_EVENTS::SETUP_TRX_DONE;
					// break;
					continue;
				}
				case 0x06:
				{
					// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_RXFLVL PKTSTS 0x06 - setup packet received");

					//SETUP data packet received
					if(get_ep_in(ep)->DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT)
					{
						flush_tx(ep);
					}

					event = USB_common::USB_EVENTS::SETUP_PACKET_RX;
					break;
				}
				default:
				{
					//???
					//force a pop
					volatile uint32_t GRXSTSP = OTG->GRXSTSP;
					continue;
				}
			}
		}
		else if(GINTSTS & USB_OTG_GINTSTS_SOF)
		{
			//uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_SOF");

			OTG->GINTSTS = USB_OTG_GINTSTS_SOF;

			event = USB_common::USB_EVENTS::SOF;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_USBSUSP)
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_USBSUSP");

			OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;

			event = USB_common::USB_EVENTS::SUSPEND;
		}
        else if(GINTSTS & USB_OTG_GINTSTS_ESUSP)
        {
        	// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_ESUSP");

            OTG->GINTSTS = USB_OTG_GINTSTS_ESUSP;
            
            event = USB_common::USB_EVENTS::EARLY_SUSPEND;
        }
		else if(GINTSTS & USB_OTG_GINTSTS_WKUINT)
		{
			// uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_WKUINT");

			OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;

			event = USB_common::USB_EVENTS::WAKEUP;
		}
		else
		{
			//no events
			return;
		}

		func(event, ep);
		break;
	}
}
#endif

void stm32_h7xx_otghs::set_data0(const uint8_t ep)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep);

	if(USB_common::is_in_ep(ep))
	{
		volatile USB_OTG_INEndpointTypeDef* epin = get_ep_in(ep_addr);

		Register_util::set_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = get_ep_out(ep_addr);

		Register_util::set_bits(&epout->DOEPCTL, USB_OTG_DOEPCTL_SD0PID_SEVNFRM);
	}
}

void stm32_h7xx_otghs::flush_rx()
{
	//verify rx read idle
	//???
	//verify rx write idle
	//???

	Register_util::set_bits(&OTG->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
	Register_util::wait_until_clear(&OTG->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
}

void stm32_h7xx_otghs::flush_tx(const uint8_t ep)
{
	//verify tx read idle
	//???
	//verify tx write idle
	Register_util::wait_until_set(&OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL);

	Register_util::wait_until_clear(&OTG->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH);

	Register_util::mask_set_bits(
		&OTG->GRSTCTL, 
		USB_OTG_GRSTCTL_TXFNUM, 
		_VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, ep) | USB_OTG_GRSTCTL_TXFFLSH
	);

	Register_util::wait_until_clear(&OTG->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH);
}

void stm32_h7xx_otghs::flush_all_tx()
{
	flush_tx(0x20);
}

const usb_driver_base::ep_cfg& stm32_h7xx_otghs::get_ep0_config() const
{
	return m_ep0_cfg;
}
bool stm32_h7xx_otghs::get_rx_ep_config(const uint8_t addr, ep_cfg* const out_ep)
{
	if(addr > MAX_NUM_EP)
	{
		return false;
	}

	*out_ep = m_tx_ep_cfg[addr - 1];
	return true;
}
bool stm32_h7xx_otghs::get_tx_ep_config(const uint8_t addr, ep_cfg* const out_ep)
{
	if(addr > MAX_NUM_EP)
	{
		return false;
	}
	
	*out_ep = m_tx_ep_cfg[addr - 1];
	return true;
}

//application waits for a buffer with data
//this might be better as a stream thing rather than buffer exchange
Buffer_adapter_base* stm32_h7xx_otghs::wait_rx_buffer(const uint8_t ep_num)
{
	return m_rx_buffer->wait_dequeue_buffer(ep_num);
}
//application returns rx buffer to driver. will allow reception to continue in event of buffer underrun
void stm32_h7xx_otghs::release_rx_buffer(const uint8_t ep_num, Buffer_adapter_base* const buf)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep_num);
	
	m_rx_buffer->release_buffer(ep_addr, buf);

	//TODO: we may need to mask usb isr here
	//it might be safe for now, since we only do this if the ep has no loaded OUT buffer
	//which means that NAK is set
	if(m_rx_buffer->get_buffer(ep_addr) == nullptr)
	{
		//clear a OUT nak, since we have a new buffer to read to

		Buffer_adapter_base* act_buf = m_rx_buffer->poll_allocate_buffer(ep_addr);
		
		m_rx_buffer->set_buffer(ep_addr, act_buf);

		get_ep_out(ep_num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}
}

//application wait for usable tx buffer
Buffer_adapter_base* stm32_h7xx_otghs::wait_tx_buffer(const uint8_t ep_num)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep_num);

	return m_tx_buffer->wait_allocate_buffer(ep_addr);
}
//application give buffer to driver for transmission
bool stm32_h7xx_otghs::enqueue_tx_buffer(const uint8_t ep_num, Buffer_adapter_base* const buf)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep_num);

	//TODO: we may need to mask usb isr here
	//it might be safe for now, since we only do this if the ep has no loaded IN buffer
	//which means that NAK is set

	if(m_tx_buffer->get_buffer(ep_addr) == nullptr)
	{
		m_tx_buffer->set_buffer(ep_addr, buf);

		ep_write(ep_num, buf->data(), buf->size());
	}
	else
	{
		if(!m_tx_buffer->poll_enqueue_buffer(ep_addr, buf))
		{
			return false;
		}
	}

	return true;
}