/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: See git blame.

	Major reductions made March 2023. William Vinnicombe.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "glaux.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"


#define							kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define							kWarpConstantStringErrorSanity		"\rSanity check failed!"

#include "devMMA8451Q.h"
volatile WarpI2CDeviceState			deviceMMA8451QState;


#include "devSSD1331.h"
volatile WarpSPIDeviceState			deviceSSD1331State;

volatile i2c_master_state_t				i2cMasterState;
volatile spi_master_state_t				spiMasterState;
volatile spi_master_user_config_t			spiUserConfig;
volatile lpuart_user_config_t				lpuartUserConfig;
volatile lpuart_state_t					lpuartState;


volatile bool						gWarpBooted				= false;
volatile uint32_t					gWarpI2cBaudRateKbps			= kWarpDefaultI2cBaudRateKbps;
volatile uint32_t					gWarpUartBaudRateBps			= kWarpDefaultUartBaudRateBps;
volatile uint32_t					gWarpSpiBaudRateKbps			= kWarpDefaultSpiBaudRateKbps;
volatile uint32_t					gWarpSleeptimeSeconds			= kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask					gWarpMode				= kWarpModeDisableAdcOnSleep;
volatile uint32_t					gWarpI2cTimeoutMilliseconds		= kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t					gWarpSpiTimeoutMicroseconds		= kWarpDefaultSpiTimeoutMicroseconds;
volatile uint32_t					gWarpUartTimeoutMilliseconds		= kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t					gWarpMenuPrintDelayMilliseconds		= kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t					gWarpSupplySettlingDelayMilliseconds	= kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t					gWarpCurrentSupplyVoltage		= kWarpDefaultSupplyVoltageMillivolts;
char							gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];

/*
 *	Since only one SPI transaction is ongoing at a time in our implementaion
 */
uint8_t							gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

static void						lowPowerPinStates(void);

#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
	static void					disableTPS62740(void);
	static void					enableTPS62740(uint16_t voltageMillivolts);
	static void					setTPS62740CommonControlLines(uint16_t voltageMillivolts);
#endif

static void						displayActivity(int delay);



/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
 */
clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
void
LLWU_IRQHandler(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *  notify, power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}
/*
 *	Derived from KSDK power_manager_demo.c <<END
 */


void
enableLPUARTpins(void)
{
	/*
	 *	Enable UART CLOCK
	 */
	CLOCK_SYS_EnableLpuartClock(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX
	 *
	 *	TODO: we don't use hw flow control so don't need RTS/CTS
 	 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
 	 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

	//TODO: we don't use hw flow control so don't need RTS/CTS
	//	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	//	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO_UART_RTS);
	//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
	 */
	lpuartUserConfig.baudRate = gWarpUartBaudRateBps;
	lpuartUserConfig.parityMode = kLpuartParityDisabled;
	lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
	lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;
	lpuartUserConfig.clockSource = kClockLpuartSrcMcgIrClk;

	LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);
}


void
disableLPUARTpins(void)
{
	/*
	 *	LPUART deinit
	 */
	LPUART_DRV_Deinit(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX
	 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
	 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	/*
	 * We don't use the HW flow control and that messes with the SPI any way
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Disable LPUART CLOCK
	 */
	CLOCK_SYS_DisableLpuartClock(0);
}



WarpStatus
sendBytesToUART(uint8_t *  bytes, size_t nbytes)
{
	lpuart_status_t	status;

	status = LPUART_DRV_SendDataBlocking(0, bytes, nbytes, gWarpUartTimeoutMilliseconds);
	if (status != 0)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



void
warpEnableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	kWarpPinSPI_MISO_UART_RTS_UART_RTS --> PTA6 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	kWarpPinSPI_MOSI_UART_CTS --> PTA7 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAlt3);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);
	#endif

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}



void
warpDisableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);

	/*	kWarpPinSPI_MISO_UART_RTS	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	kWarpPinSPI_MOSI_UART_CTS	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	#endif

	//TODO: we don't use HW flow control so can remove these since we don't use the RTS/CTS
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	CLOCK_SYS_DisableSpiClock(0);
}

void
warpEnableI2Cpins(void)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	(ALT2 == I2C)
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	(ALT2 == I2C)
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}

void
lowPowerPinStates(void)
{
	/*
	 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
	 *	we configure all pins as output and set them to a known state. We choose
	 *	to set them all to '0' since it happens that the devices we want to keep
	 *	deactivated (SI4705) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
	 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
	 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
	 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
	 *	functionality.
	 *
	 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	/*
	 *	Disable PTA5
	 *
	 *	NOTE: Enabling this significantly increases current draw
	 *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
	 *
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

	/*
	 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
	 *
	 *		"Unused pins should be configured in the disabled state, mux(0),
	 *		to prevent unwanted leakage (potentially caused by floating inputs)."
	 *
	 *	However, other documents advice to place pin as GPIO and drive low or high.
	 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);


	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
}

void
warpPrint(const char *fmt, ...)
{
	int	fmtlen;
	va_list	arg;

	/*
	 *	We use an ifdef rather than a C if to allow us to compile-out
	 *	all references to SEGGER_RTT_*printf if we don't want them.
	 *
	 *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
	 *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
	 *	also takes our print buffer which we will eventually send over
	 *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
	 *	2kB flash and removes the use of malloc so we can keep heap
	 *	allocation to zero.
	 */

	/*
	 *	We can't use SEGGER_RTT_vprintf to format into a buffer
	 *	since SEGGER_RTT_vprintf formats directly into the special
	 *	RTT memory region to be picked up by the RTT / SWD mechanism...
	 */
	va_start(arg, fmt);
	fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
	va_end(arg);

	if (fmtlen < 0)
	{
		SEGGER_RTT_WriteString(0, gWarpEfmt);

		return;
	}

	/*
	 *	Throttle to enable SEGGER to grab output, otherwise "run" mode may miss lines.
	 */
	OSA_TimeDelay(5);

	return;
}

int
main(void)
{
	WarpStatus				status;
	rtc_datetime_t				warpBootDate;
	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	/*
	 *	We use this as a template later below and change the .mode fields for the different other modes.
	 */
	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: POWER_SYS_SetMode() depends on this order
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure		powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};

	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);

	/*
	 *	Set board crystal value (Warp revB and earlier).
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Initialize RTC Driver (not needed on Glaux, but we enable it anyway for now
	 *	as that lets us use the current sleep routines). NOTE: We also don't seem to
	 *	be able to go to VLPR mode unless we enable the RTC.
	 */
	RTC_DRV_Init(0);

	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);

	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);

	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	if (WARP_BUILD_BOOT_TO_VLPR)
	{
		warpPrint("About to switch CPU to VLPR mode... ");
		status = warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
		if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
		{
			warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR() failed...\n");
		}
		warpPrint("done.\n\r");
	}

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	warpPrint("About to GPIO_DRV_Init()... ");
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	warpPrint("done.\n");

	/*
	 *	Make sure the SWD pins, PTA0/1/2 SWD pins in their ALT3 state (i.e., as SWD).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	warpPrint("About to lowPowerPinStates()... ");
	lowPowerPinStates();
	warpPrint("done.\n");

	#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
		initMMA8451Q(	0x1D	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsMMA8451Q	);
	#endif

	devSSD1331init();

	/*
	 *	At this point, we consider the system "booted" and, e.g., warpPrint()s
	 *	will also be sent to the BLE if that is compiled in.
	 */
	gWarpBooted = true;
	warpPrint("Boot done.\n");

	while (1)
	{
		displayActivity(10000);
	}

	return 0;
}

/*
	Code Below Authored March 2023. William Vinnicombe.
 */

void displayActivity(int delay)
{
	configureSensorMMA8451Q(0x00,/* Payload: Disable FIFO */
					0x01/* Normal read 8bit, 800Hz, normal, active mode */
					);

	while(true) {
		uint32_t start = OSA_TimeGetMsec();

		#define BUFFER_SIZE 100

		int16_t xAccels[BUFFER_SIZE] = {0};
		int16_t yAccels[BUFFER_SIZE] = {0};
		int16_t zAccels[BUFFER_SIZE] = {0};

		int i = 0;

		int interval = delay/BUFFER_SIZE;

		while (OSA_TimeGetMsec() < start + delay)
		{
			uint16_t	readSensorRegisterValueLSB;
			uint16_t	readSensorRegisterValueMSB;
			int16_t		xAccel;
			int16_t		yAccel;
			int16_t		zAccel;
			WarpStatus	i2cReadStatus;
			i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
			bool invalid = false;
			if (i2cReadStatus == kWarpStatusOK)
			{
				readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
				readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
				xAccel = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
				xAccel = (xAccel ^ (1 << 13)) - (1 << 13);
				xAccels[i] = xAccel;
			} else {
				invalid = true;
			}

			i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
			if (i2cReadStatus == kWarpStatusOK)
			{
				readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
				readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
				yAccel = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
				yAccel = (yAccel ^ (1 << 13)) - (1 << 13);
				yAccels[i] = yAccel;
			} else {
				invalid = true;
			}

			i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
			if (i2cReadStatus == kWarpStatusOK)
			{
				readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
				readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
				zAccel = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
				zAccel = (zAccel ^ (1 << 13)) - (1 << 13);
				zAccels[i] = zAccel;
			} else {
				invalid = true;
			}

			// warpPrint("x: %d, y; %d, z: %d\n", xAccel, yAccel, zAccel);
			if (!invalid)
				i++;
			else
				warpPrint("Invalid I2C Read");
			if (i > BUFFER_SIZE) {
				break;
			}
			while ((start + interval*i) > OSA_TimeGetMsec())
			{}
		}
		warpPrint("i: %d\n", i);

		int sitCount = 0;
		int standCount = 0;
		int walkCount = 0;
		int runCount = 0;
		int unknownCount = 0;

		int movementThreshold = 500;
		int runThreshold = 4000;
		for (int j=1; j < i; j++) {
			int16_t x = xAccels[j];
			int16_t y = yAccels[j];
			int16_t z = zAccels[j];

			int16_t dx = x - xAccels[j-1];
			int16_t dy = y - yAccels[j-1];
			int16_t dz = z - zAccels[j-1];

			if (j%10 == 0)
			{
				warpPrint("x: %d, y: %d, z: %d\n", x, y, z);
				warpPrint("dx: %d, dy: %d, dz: %d\n", dx, dy, dz);
			}

			if (abs(dx) + abs(dy) + abs(dz) < movementThreshold)
			{
				if (abs(y) < abs(x) + abs(z))
				{
					if (abs(z) > abs(x))
						sitCount += 1;
					else
						standCount += 1;
				}
				else
				{
					unknownCount += 1;
				}
			}
			else
			{
				if (abs(dx) + abs(dy) + abs(dz) > runThreshold)
					runCount += 1;
				else
					walkCount += 1;
			}
		}
		warpPrint("Those were saved\n");
		warpPrint("Sit %d, Stand %d, Walk %d, Run %d, Unknown %d\n", sitCount, standCount, walkCount, runCount, unknownCount);
		SSD1331Drawings activity = kSSD1331DrawingNone;
		int maxVal = max(max(max(sitCount, standCount), max(walkCount, runCount)), unknownCount);
		if (maxVal == sitCount)
			activity = kSSD1331DrawingSit;
		else if (maxVal == standCount)
			activity = kSSD1331DrawingStand;
		else if (maxVal == walkCount)
			activity = kSSD1331DrawingWalk;
		else if (maxVal == runCount)
			activity = kSSD1331DrawingRun;
		else
			activity = kSSD1331DrawingNone;
		float certainty = (float)maxVal/99.0;
		devSSD1331text(activity);
		devSSD1331bar(certainty);
	}
}
