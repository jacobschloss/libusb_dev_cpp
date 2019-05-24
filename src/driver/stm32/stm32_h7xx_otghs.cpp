/**
 * @brief stm32_h7xx_otghs
 * @author Jacob Schloss <jacob@schloss.io>
 * @copyright Copyright (c) 2019 Jacob Schloss. All rights reserved.
 * @license Licensed under the 3-Clause BSD license. See LICENSE for details
*/

#include "libusb_dev_cpp/driver/stm32/stm32_h7xx_otghs.hpp"

#include "libusb_dev_cpp/driver/cpu/Cortex_m7.hpp"

#include "STM32H7xx/Include/stm32h7xx.h"

#include "common_util/Byte_util.hpp"
#include "common_util/Register_util.hpp"

#include "uart1_printf.hpp"

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

}
stm32_h7xx_otghs::~stm32_h7xx_otghs()
{

}

bool stm32_h7xx_otghs::initialize()
{
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
	//wait for USB1 to be idle
	Register_util::wait_until_set(&OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL);

	//soft reset
	Register_util::set_bits(&OTG->GRSTCTL,         USB_OTG_GRSTCTL_CSRST);
	Register_util::wait_until_clear(&OTG->GRSTCTL, USB_OTG_GRSTCTL_CSRST);
	//wait for USB1 to be idle
	Register_util::wait_until_set(&OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL);

	//config as device
	Register_util::clear_bits(&OTG->GUSBCFG, 
		USB_OTG_GUSBCFG_PTCI       | 
		USB_OTG_GUSBCFG_PCCI       | 
		USB_OTG_GUSBCFG_ULPIEVBUSI | 
		USB_OTG_GUSBCFG_ULPIEVBUSD | 
		USB_OTG_GUSBCFG_ULPICSM    | 
		USB_OTG_GUSBCFG_ULPIAR     | 
		USB_OTG_GUSBCFG_ULPIFSLS   | 
		USB_OTG_GUSBCFG_PHYLPCS    | 
		USB_OTG_GUSBCFG_HNPCAP     | 
		USB_OTG_GUSBCFG_SRPCAP     | 
		USB_OTG_GUSBCFG_PHYSEL
		);
	Register_util::set_bits(&OTG->GUSBCFG, USB_OTG_GUSBCFG_FDMOD);
	Register_util::mask_set_bits(&OTG->GUSBCFG, USB_OTG_GUSBCFG_TRDT, _VAL2FLD(USB_OTG_GUSBCFG_TRDT,   0x09));
	Register_util::mask_set_bits(&OTG->GUSBCFG, USB_OTG_GUSBCFG_TOCAL, _VAL2FLD(USB_OTG_GUSBCFG_TOCAL, 0x05));

	//power down
	Register_util::clear_bits(&OTG->GCCFG, (1U << 20) | (1U << 19) | (1U << 18) | (1U << 17) | USB_OTG_GCCFG_PWRDWN);
	//No vbus sense
	Register_util::set_bits<uint32_t>(&OTG->GCCFG, 1U << 21);

	Register_util::mask_set_bits(&OTGD->DCFG, USB_OTG_DCFG_DSPD,      _VAL2FLD(USB_OTG_DCFG_DSPD,      0x00));
	Register_util::mask_set_bits(&OTGD->DCFG, USB_OTG_DCFG_DAD,       _VAL2FLD(USB_OTG_DCFG_DAD,       0x00));
	Register_util::mask_set_bits(&OTGD->DCFG, USB_OTG_DCFG_PFIVL,     _VAL2FLD(USB_OTG_DCFG_PFIVL,     0x00));
	Register_util::mask_set_bits(&OTGD->DCFG, USB_OTG_DCFG_PERSCHIVL, _VAL2FLD(USB_OTG_DCFG_PERSCHIVL, 0x00));
	Register_util::set_bits(&OTGD->DCFG, USB_OTG_DCFG_NZLSOHSK);

	//soft disconnect
	Register_util::set_bits(&OTGD->DCTL, USB_OTG_DCTL_SDIS);

	//reset fifo assignments
	for (size_t i = 1; i < MAX_NUM_EP; i++)
	{
		OTG->DIEPTXF[i-1] = _VAL2FLD(USB_OTG_DIEPTXF_INEPTXFD, 0x0200) | _VAL2FLD(USB_OTG_DIEPTXF_INEPTXSA, 0x0200+0x0200*i);
	}

	//rx fifo
	OTG->GRXFSIZ = RX_FIFO_SIZE;
	//ep0 tx fifo, TX0FD | TX0FSA
	if(!config_ep_tx_fifo(0, 64))
	{
		return false;
	}
	// OTG->DIEPTXF0_HNPTXFSIZ = Byte_util::make_u32(0x0010, RX_FIFO_SIZE);

	//tx ep int
	OTGD->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;

	//start clocks, no sleep gate
	Register_util::clear_bits(OTGPCTL, USB_OTG_PCGCR_PHYSUSP | USB_OTG_PCGCR_GATEHCLK | USB_OTG_PCGCR_STPPCLK);

	//clear core interrupt
	OTG->GINTMSK = 0U;
	OTG->GINTSTS = 0xFFFFFFFF;

	//config core interrupt
	OTG->GINTMSK  = USB_OTG_GINTMSK_USBRST   |
					USB_OTG_GINTMSK_ENUMDNEM |
    				USB_OTG_GINTMSK_USBSUSPM |
					USB_OTG_GINTMSK_ESUSPM   |
    				//USB_OTG_GINTMSK_SOFM   |
					USB_OTG_GINTMSK_WUIM     |
					USB_OTG_GINTMSK_IEPINT   |
					USB_OTG_GINTMSK_RXFLVLM  ;
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
	OTGD->DCFG |= _VAL2FLD(USB_OTG_DCFG_DAD, addr);

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
		ep_cfg ep0_cfg = ep;
		if(ep.size <= 8)
		{
			ep0_cfg.size = 8;
			mpsize = 3;
		}
		else if(ep.size <= 16)
		{
			ep0_cfg.size = 16;
			mpsize = 2;
		}
		else if(ep.size <= 32)
		{
			ep0_cfg.size = 32;
			mpsize = 1;
		}
		else
		{
			ep0_cfg.size = 64;
			mpsize = 0;
		}

		OTGD->DAINTMSK |= 0x00010001;

		ep_in->DIEPCTL = 
			USB_OTG_DIEPCTL_SNAK | 
			_VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, ep_addr) | 
			_VAL2FLD(USB_OTG_DIEPCTL_EPTYP, 0x00)     | 
			_VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, ep0_cfg.size);
		
		ep_out->DOEPTSIZ = 
			_VAL2FLD(USB_OTG_DOEPTSIZ_STUPCNT, 1) |
			USB_OTG_DOEPTSIZ_PKTCNT               |
			_VAL2FLD(USB_OTG_DOEPTSIZ_XFRSIZ, ep0_cfg.size);
		
		ep_out->DOEPCTL = 
			USB_OTG_DOEPCTL_EPENA | 
			USB_OTG_DOEPCTL_CNAK  | 
			_VAL2FLD(USB_OTG_DOEPCTL_MPSIZ, mpsize);
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
					USB_OTG_DIEPCTL_USBAEP                    | 
					USB_OTG_DIEPCTL_SNAK                      | 
					_VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, ep_addr) | 
					_VAL2FLD(USB_OTG_DIEPCTL_EPTYP, 0x02)     | 
					USB_OTG_DIEPCTL_SD0PID_SEVNFRM            | 
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
					USB_OTG_DIEPCTL_CNAK                      | 
					USB_OTG_DIEPCTL_USBAEP                    | 
					_VAL2FLD(USB_OTG_DOEPCTL_EPTYP, 0x02)     | 
					USB_OTG_DIEPCTL_SD0PID_SEVNFRM            | 
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

		Register_util::set_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_STALL);
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = get_ep_out(ep_addr);

		Register_util::set_bits(&epout->DOEPCTL, USB_OTG_DOEPCTL_STALL);
	}
}
void stm32_h7xx_otghs::ep_unstall(const uint8_t ep)
{
	const uint8_t ep_addr = USB_common::get_ep_addr(ep);

	if(USB_common::is_in_ep(ep))
	{
		volatile USB_OTG_INEndpointTypeDef* epin = get_ep_in(ep_addr);

		Register_util::clear_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_STALL);
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = get_ep_out(ep_addr);

		Register_util::clear_bits(&epout->DOEPCTL, USB_OTG_DOEPCTL_STALL);
	}
}

size_t stm32_h7xx_otghs::ep_write(const uint8_t ep, const uint8_t* buf, const uint16_t len)
{
	if(USB_common::is_in_ep(ep))
	{
		return 0;
	}

	const uint8_t ep_addr = USB_common::get_ep_addr(ep);

	volatile uint32_t* const fifo = get_ep_fifo(ep_addr);
	volatile USB_OTG_INEndpointTypeDef* const epin = get_ep_in(ep_addr);

	const size_t len32 = (len + 3) / 4;

	//number of words availible
	const uint32_t DTXFSTS = epin->DTXFSTS;
	const uint32_t INEPTFSAV = _FLD2VAL(USB_OTG_DTXFSTS_INEPTFSAV, DTXFSTS);
	if(INEPTFSAV < len32)
	{
		return 0;
	}

	Register_util::mask_set_bits(
						&epin->DIEPCTL,
						USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_MULCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
						_VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_MULCNT, 1 ) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, len)
					);
	Register_util::mask_set_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_STALL, USB_OTG_DOEPCTL_CNAK);
	Register_util::set_bits(&epin->DIEPCTL, USB_OTG_DOEPCTL_EPENA);

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
					break;
				}
			}

			*fifo = Byte_util::make_u32(0, b2, b1, b0);
		}
	}

	return 0;
}
size_t stm32_h7xx_otghs::ep_read(const uint8_t ep, uint8_t* const buf, const uint16_t max_len)
{
	//no data
	if(!(OTG->GINTSTS & USB_OTG_GINTSTS_RXFLVL))
	{
		return 0;
	}

	//no data for that ep
	if((OTG->GRXSTSR & USB_OTG_GRXSTSP_EPNUM) != ep)
	{
		return 0;
	}

	const uint32_t GRXSTSP = OTG->GRXSTSP;
	const size_t blen = std::min<uint32_t>(_FLD2VAL(USB_OTG_GRXSTSP_BCNT, GRXSTSP), max_len);

	for(size_t i = 0; i < blen; i += 4)
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

	return blen;
}

uint16_t stm32_h7xx_otghs::get_frame_number()
{
	return _FLD2VAL(USB_OTG_DSTS_FNSOF, OTGD->DSTS);
}
size_t stm32_h7xx_otghs::get_serial_number(uint8_t* const buf, const size_t maxlen)
{
	return 0;
}

void stm32_h7xx_otghs::poll(const USB_common::Event_callback& func)
{
	USB_common::USB_EVENTS event = USB_common::USB_EVENTS::RESET;
	uint8_t ep = 0;


	while(true)
	{
		const uint32_t GINTSTS = OTG->GINTSTS;

		if(GINTSTS & USB_OTG_GINTSTS_USBRST)
		{
			uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_USBRST");

			OTG->GINTSTS = USB_OTG_GINTSTS_USBRST;

			for(uint8_t i = 0; i < MAX_NUM_EP; i++)
			{
				ep_unconfig(i);
			}

			flush_rx();

			continue;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
		{
			uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_ENUMDNE");

			OTG->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;

			event = USB_common::USB_EVENTS::RESET;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_IEPINT)
		{
			uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_IEPINT");
		}
		else if(GINTSTS & USB_OTG_GINTSTS_RXFLVL)
		{
			uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_RXFLVL");

			const uint32_t GRXSTSR = OTG->GRXSTSR;

			ep = _FLD2VAL(USB_OTG_GRXSTSP_EPNUM, GRXSTSR);
			switch(_FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, GRXSTSR))
			{
				case 0x01:
				{
					//GLOBAL OUT NAK, int
					break;
				}
				case 0x02:
				{
					//OUT packet received
					event = USB_common::USB_EVENTS::EPRX;
					break;
				}
				case 0x03:
				{
					//OUT transfer completed, int
					Register_util::set_bits(&(get_ep_out(ep)->DOEPCTL), USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
					volatile uint32_t GRXSTSP = OTG->GRXSTSP;
					continue;
				}
				case 0x04:
				{
					uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_RXFLVL PKTSTS 0x04");

					//SETUP transaction completed, int
					Register_util::set_bits(&(get_ep_out(ep)->DOEPCTL), USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
					volatile uint32_t GRXSTSP = OTG->GRXSTSP;
					continue;
				}
				case 0x06:
				{
					uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_RXFLVL PKTSTS 0x06");

					//SETUP data packet received
					if(get_ep_in(ep)->DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT)
					{
						flush_tx(ep);
					}

					event = USB_common::USB_EVENTS::EPSETUP;
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
			uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_USBSUSP");

			OTG->GINTSTS = USB_OTG_GINTSTS_SOF;

			event = USB_common::USB_EVENTS::SOF;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_USBSUSP)
		{
			uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_USBSUSP");

			OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;

			event = USB_common::USB_EVENTS::SUSPEND;
		}
        else if(GINTSTS & USB_OTG_GINTSTS_ESUSP)
        {
        	uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_ESUSP");

            OTG->GINTSTS = USB_OTG_GINTSTS_ESUSP;
            continue;
        }
		else if(GINTSTS & USB_OTG_GINTSTS_WKUINT)
		{
			uart1_log<64>(LOG_LEVEL::INFO, "stm32_h7xx_otghs", "USB_OTG_GINTSTS_WKUINT");

			OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;

			event = USB_common::USB_EVENTS::WAKEUP;
		}
		else
		{
			//no events
			break;
		}

		func(event, ep);
		break;
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