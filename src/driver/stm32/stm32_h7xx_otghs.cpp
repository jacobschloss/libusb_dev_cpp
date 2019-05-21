#include "libusb_dev_cpp/driver/stm32/stm32_h7xx_otghs.hpp"

#include "libusb_dev_cpp/driver/cpu/Cortex_m7.hpp"

#include "STM32H7xx/Include/stm32h7xx.h"

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
	//reset usb if
	if(RCC->AHB1ENR & (RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN))
	{
		if(!disable())
		{
			return false;
		}
	}

	//enable usb core and ulpi clock for USB1
	_BST(RCC->AHB1ENR, RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN);

	//wait for USB1 to be idle
	_WBS(OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL);

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
