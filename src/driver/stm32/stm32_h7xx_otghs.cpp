#include "libusb_dev_cpp/driver/stm32/stm32_h7xx_otghs.hpp"

#include "libusb_dev_cpp/driver/cpu/Cortex_m7.hpp"

#include "STM32H7xx/Include/stm32h7xx.h"

#include "common_util/Byte_util.hpp"
#include "common_util/Register_util.hpp"

#include "uart1_printf.hpp"

namespace
{
	static constexpr size_t MAX_EP = 8;
	static constexpr size_t MAX_RX_PACKET = 512;
	static constexpr size_t MAX_FIFO_SZ = 1024; //uint32 * 1024

	static volatile USB_OTG_GlobalTypeDef* const OTG  = reinterpret_cast<volatile USB_OTG_GlobalTypeDef*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
	static volatile USB_OTG_DeviceTypeDef* const OTGD = reinterpret_cast<volatile USB_OTG_DeviceTypeDef*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
	static volatile uint32_t* const OTGPCTL          = reinterpret_cast<volatile uint32_t*>(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE);

	static inline volatile uint32_t* EPFIFO(const uint8_t ep)
	{
		return (volatile uint32_t*)(USB1_OTG_HS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep * USB_OTG_FIFO_SIZE));
	}
	static inline volatile USB_OTG_INEndpointTypeDef* EPIN(const uint8_t ep)
	{
		return (volatile USB_OTG_INEndpointTypeDef*)(USB1_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep * USB_OTG_EP_REG_SIZE));
	}
	static inline volatile USB_OTG_OUTEndpointTypeDef* EPOUT(const uint8_t ep)
	{
		return (volatile USB_OTG_OUTEndpointTypeDef*)(USB1_OTG_HS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep * USB_OTG_EP_REG_SIZE));
	}	
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
	for (size_t i = 0; i < 15; i++)
	{
		OTG->DIEPTXF[i] = 0;
	}

	//rx fifo
	OTG->GRXFSIZ = RX_FIFO_SIZE;
	//ep0 tx fifo
	OTG->DIEPTXF0_HNPTXFSIZ = RX_FIFO_SIZE | (0x10 << 16);

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

bool stm32_h7xx_otghs::ep_setup(const uint8_t ep)
{
	return 0;
}
bool stm32_h7xx_otghs::ep_config(const uint8_t ep)
{
	return 0;
}
void stm32_h7xx_otghs::ep_unconfig(const uint8_t ep)
{
	
}

bool stm32_h7xx_otghs::ep_is_stalled(const uint8_t ep)
{
	bool is_stalled = false;

	if(USB_common::is_in_ep(ep))
	{
		volatile USB_OTG_INEndpointTypeDef* epin = EPIN(USB_common::get_ep_addr(ep));

		is_stalled = epin->DIEPCTL & USB_OTG_DIEPCTL_STALL;
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = EPOUT(USB_common::get_ep_addr(ep));

		is_stalled = epout->DOEPCTL | USB_OTG_DOEPCTL_STALL;
	}

	return is_stalled;
}
void stm32_h7xx_otghs::ep_stall(const uint8_t ep)
{
	if(USB_common::is_in_ep(ep))
	{
		volatile USB_OTG_INEndpointTypeDef* epin = EPIN(USB_common::get_ep_addr(ep));

		Register_util::set_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_STALL);
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = EPOUT(USB_common::get_ep_addr(ep));

		Register_util::set_bits(&epout->DOEPCTL, USB_OTG_DOEPCTL_STALL);
	}
}
void stm32_h7xx_otghs::ep_unstall(const uint8_t ep)
{
	if(USB_common::is_in_ep(ep))
	{
		volatile USB_OTG_INEndpointTypeDef* epin = EPIN(USB_common::get_ep_addr(ep));

		Register_util::clear_bits(&epin->DIEPCTL, USB_OTG_DIEPCTL_STALL);
	}
	else
	{
		volatile USB_OTG_OUTEndpointTypeDef* epout = EPOUT(USB_common::get_ep_addr(ep));

		Register_util::clear_bits(&epout->DOEPCTL, USB_OTG_DOEPCTL_STALL);
	}
}

size_t stm32_h7xx_otghs::ep_write(const uint8_t ep, const uint8_t* buf, const uint16_t len)
{
	volatile uint32_t* fifo = EPFIFO(ep);

	const size_t len32 = (len + 3) / 4;

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
size_t stm32_h7xx_otghs::ep_read(const uint8_t ep, uint8_t* const buf, const uint16_t len)
{
	return 0;
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

			for(uint8_t i = 0; i < NUM_EP; i++)
			{
				ep_unconfig(i);
			}

			flush_rx();

			continue;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
		{
			OTG->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;

			event = USB_common::USB_EVENTS::RESET;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_IEPINT)
		{

		}
		else if(GINTSTS & USB_OTG_GINTSTS_RXFLVL)
		{

		}
		else if(GINTSTS & USB_OTG_GINTSTS_SOF)
		{
			OTG->GINTSTS = USB_OTG_GINTSTS_SOF;

			event = USB_common::USB_EVENTS::SOF;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_USBSUSP)
		{
			OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;

			event = USB_common::USB_EVENTS::SUSPEND;
		}
		else if(GINTSTS & USB_OTG_GINTSTS_WKUINT)
		{
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
	Register_util::set_bits(&OTG->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
	Register_util::wait_until_clear(&OTG->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
}

void stm32_h7xx_otghs::flush_tx(const uint8_t ep)
{
	Register_util::mask_set_bits(
		&OTG->GRSTCTL, 
		USB_OTG_GRSTCTL_TXFNUM, 
		_VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, ep) | USB_OTG_GRSTCTL_TXFFLSH
	);

	Register_util::wait_until_clear(&OTG->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH);
}