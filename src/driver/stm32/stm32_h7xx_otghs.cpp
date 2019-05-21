#include "libusb_dev_cpp/driver/stm32/stm32_h7xx_otghs.hpp"

#include "libusb_dev_cpp/driver/cpu/Cortex_m7.hpp"

#include "STM32H7xx/Include/stm32h7xx.h"

#include "common_util/Byte_util.hpp"
#include "common_util/Register_util.hpp"

namespace
{
	static constexpr size_t MAX_EP = 8;
	static constexpr size_t MAX_RX_PACKET = 512;
	static constexpr size_t MAX_FIFO_SZ = 1024; //uint32 * 1024

	static volatile USB_OTG_GlobalTypeDef* const OTG  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
	static volatile USB_OTG_DeviceTypeDef* const OTGD = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
	static volatile uint32_t * const OTGPCTL  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE);

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
	Register_util::set_bits(OTG->GRSTCTL, USB_OTG_GRSTCTL_CSRST);
	Register_util::wait_until_clear(OTG->GRSTCTL, USB_OTG_GRSTCTL_CSRST);

	//No PD, no internal transceiver
	OTG->GCCFG = 0;

	//???
	Register_util::clear_bits(OTG->GUSBCFG, USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

	//???
	OTG->GUSBCFG = USB_OTG_GUSBCFG_FDMOD                 | 
	               //USB_OTG_GUSBCFG_ULPIIPD               |
	               _VAL2FLD(USB_OTG_GUSBCFG_TRDT,  0x09) |
	               _VAL2FLD(USB_OTG_GUSBCFG_TOCAL, 0x01) ;

	//reset fifo assignments
	for (size_t i = 0; i < 15; i++)
	{
		OTG->DIEPTXF[i] = 0;
	}

	//start clocks, no sleep gate
	*OTGPCTL = 0;

	//USB HS mode
	_BMD(OTGD->DCFG, USB_OTG_DCFG_DSPD, _VAL2FLD(USB_OTG_DCFG_DSPD, 0x00));

	//???
    OTGD->DCFG |= USB_OTG_DCFG_NZLSOHSK;

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
	Register_util::set_bits(OTG->GAHBCFG, USB_OTG_GAHBCFG_GINT);

	return true;
}
bool stm32_h7xx_otghs::disable()
{
	if(RCC->AHB1ENR & (RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN))
	{
		//reset USB1
		_BST(RCC->AHB1RSTR, RCC_AHB1RSTR_USB1OTGHSRST);
		_BCL(RCC->AHB1RSTR, RCC_AHB1RSTR_USB1OTGHSRST);

		//gate clocks
		_BCL(RCC->AHB1ENR, RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN);

		//flush
		Cortex_m7::data_instruction_sync();
	}

	return true;
}

bool stm32_h7xx_otghs::connect()
{
	_BCL(OTGD->DCTL, USB_OTG_DCTL_SDIS);

	return true;
}
bool stm32_h7xx_otghs::disconnect()
{
	_BST(OTGD->DCTL, USB_OTG_DCTL_SDIS);

	//flush
	Cortex_m7::data_instruction_sync();

	return true;
}

bool stm32_h7xx_otghs::set_usb_address(const uint8_t addr)
{
	OTGD->DCFG |= _VAL2FLD(USB_OTG_DCFG_DAD, addr);
}

size_t ep_write(const uint8_t ep, const uint8_t* buf, const uint16_t len)
{
	volatile uint32_t* fifo = EPFIFO(ep);

	const size_t len32 = (blen + 3) / 4;

	std::array<uint8_t, 4> temp_4u8;
	for(size_t i = 0 i < len; i+=4)
	{
		//copy all 4 if avail, otherwise only the remainint
		if((i+4) <= len)
		{
			temp_4u8[0] = buf[i + 0];
			temp_4u8[1] = buf[i + 1];
			temp_4u8[2] = buf[i + 2];
			temp_4u8[3] = buf[i + 3];

			*fifo = Byte_util::make_u32(b3, b2, b1, b0);
		}
	}
}
size_t ep_read(const uint8_t ep, uint8_t* const buf, const uint16_t len)
{

}