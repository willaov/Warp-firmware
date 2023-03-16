#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
// enum
// {
// 	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
// 	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
// 	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
// 	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
// 	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
// };

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}



int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x09);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");

	return 0;
}


int
devSSD1331green(void)
{
	/*
	 *	Any post-initialization drawing commands go here.
	 */
	writeCommand(kSSD1331CommandDRAWRECT);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	writeCommand(0x00);
	writeCommand(0xFF);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x7F);
	writeCommand(0x00);
	SEGGER_RTT_WriteString(0, "\r\n\tGreen rectangle...\n");

	return 0;
}


int
devSSD1331red(void)
{
	/*
	 *	Any post-initialization drawing commands go here.
	 */
	writeCommand(kSSD1331CommandDRAWRECT);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	writeCommand(0xFF);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x7F);
	writeCommand(0x00);
	writeCommand(0x00);
	SEGGER_RTT_WriteString(0, "\r\n\tRed rectangle...\n");

	return 0;
}


int
devSSD1331blue(void)
{
	/*
	 *	Any post-initialization drawing commands go here.
	 */
	writeCommand(kSSD1331CommandDRAWRECT);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0xFF);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x7F);
	SEGGER_RTT_WriteString(0, "\r\n\tBlue rectangle...\n");

	return 0;
}


int
devSSD1331colour(uint8_t red, uint8_t green, uint8_t blue)
{
	/*
	 *	Any post-initialization drawing commands go here.
	 */
	writeCommand(kSSD1331CommandDRAWRECT);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	writeCommand(red);
	writeCommand(green);
	writeCommand(blue);
	writeCommand(red);
	writeCommand(green);
	writeCommand(blue);
	SEGGER_RTT_WriteString(0, "\r\n\tColour rectangle...\n");

	return 0;
}


int devSSD1331text(SSD1331Drawings picture)
{
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	switch (picture)
	{
	case kSSD1331DrawingStand:
		writeCommand(kSSD1331CommandDRAWRECT);	// Head
		writeCommand(12+33);
		writeCommand(5+12);
		writeCommand(18+33);
		writeCommand(11+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0x00);
		writeCommand(0x00);
		writeCommand(0x00);

		writeCommand(kSSD1331CommandDRAWLINE); // Body
		writeCommand(15+33);
		writeCommand(11+12);
		writeCommand(15+33);
		writeCommand(26+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Arms
		writeCommand(9+33);
		writeCommand(17+12);
		writeCommand(21+33);
		writeCommand(17+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Right Leg
		writeCommand(15+33);
		writeCommand(26+12);
		writeCommand(20+33);
		writeCommand(34+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Left Leg
		writeCommand(15+33);
		writeCommand(26+12);
		writeCommand(10+33);
		writeCommand(34+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		break;
	
	case kSSD1331DrawingSit:
		writeCommand(kSSD1331CommandDRAWRECT);	// Head
		writeCommand(11+33);
		writeCommand(8+12);
		writeCommand(17+33);
		writeCommand(14+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0x00);
		writeCommand(0x00);
		writeCommand(0x00);

		writeCommand(kSSD1331CommandDRAWLINE); // Body
		writeCommand(13+33);
		writeCommand(14+12);
		writeCommand(13+33);
		writeCommand(24+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Arm
		writeCommand(13+33);
		writeCommand(18+12);
		writeCommand(19+33);
		writeCommand(21+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Upper Leg
		writeCommand(13+33);
		writeCommand(24+12);
		writeCommand(19+33);
		writeCommand(24+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Lower Leg
		writeCommand(19+33);
		writeCommand(24+12);
		writeCommand(19+33);
		writeCommand(30+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		break;

	case kSSD1331DrawingWalk:
		writeCommand(kSSD1331CommandDRAWRECT);	// Head
		writeCommand(12+33);
		writeCommand(5+12);
		writeCommand(18+33);
		writeCommand(11+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0x00);
		writeCommand(0x00);
		writeCommand(0x00);

		writeCommand(kSSD1331CommandDRAWLINE); // Body
		writeCommand(14+33);
		writeCommand(11+12);
		writeCommand(14+33);
		writeCommand(23+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // UR Arm
		writeCommand(14+33);
		writeCommand(15+12);
		writeCommand(16+33);
		writeCommand(18+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // LR Arm
		writeCommand(16+33);
		writeCommand(18+12);
		writeCommand(19+33);
		writeCommand(20+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // UL Arm
		writeCommand(14+33);
		writeCommand(15+12);
		writeCommand(12+33);
		writeCommand(18+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // LL Arm
		writeCommand(12+33);
		writeCommand(18+12);
		writeCommand(11+33);
		writeCommand(21+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // UR Leg
		writeCommand(14+33);
		writeCommand(23+12);
		writeCommand(17+33);
		writeCommand(25+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // LR Leg
		writeCommand(17+33);
		writeCommand(25+12);
		writeCommand(18+33);
		writeCommand(29+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // UL Leg
		writeCommand(14+33);
		writeCommand(23+12);
		writeCommand(13+33);
		writeCommand(26+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // LL Leg
		writeCommand(13+33);
		writeCommand(26+12);
		writeCommand(11+33);
		writeCommand(29+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		break;

	case kSSD1331DrawingRun:
		writeCommand(kSSD1331CommandDRAWRECT);	// Head
		writeCommand(17+33);
		writeCommand(4+12);
		writeCommand(23+33);
		writeCommand(10+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0x00);
		writeCommand(0x00);
		writeCommand(0x00);

		writeCommand(kSSD1331CommandDRAWLINE); // Body
		writeCommand(19+33);
		writeCommand(10+12);
		writeCommand(15+33);
		writeCommand(21+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // UR Arm
		writeCommand(18+33);
		writeCommand(13+12);
		writeCommand(20+33);
		writeCommand(15+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // LR Arm
		writeCommand(20+33);
		writeCommand(15+12);
		writeCommand(23+33);
		writeCommand(14+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // UL Arm
		writeCommand(18+33);
		writeCommand(13+12);
		writeCommand(15+33);
		writeCommand(14+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // LL Arm
		writeCommand(15+33);
		writeCommand(14+12);
		writeCommand(14+33);
		writeCommand(17+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // UR Leg
		writeCommand(15+33);
		writeCommand(21+12);
		writeCommand(18+33);
		writeCommand(23+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // LR Leg
		writeCommand(18+33);
		writeCommand(23+12);
		writeCommand(19+33);
		writeCommand(27+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // UL Leg
		writeCommand(15+33);
		writeCommand(21+12);
		writeCommand(12+33);
		writeCommand(23+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // LL Leg
		writeCommand(12+33);
		writeCommand(23+12);
		writeCommand(9+33);
		writeCommand(23+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		break;
	
	default:
		writeCommand(kSSD1331CommandDRAWLINE); // Left
		writeCommand(10+33);
		writeCommand(15+12);
		writeCommand(10+33);
		writeCommand(10+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Top
		writeCommand(10+33);
		writeCommand(10+12);
		writeCommand(20+33);
		writeCommand(10+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Right
		writeCommand(20+33);
		writeCommand(10+12);
		writeCommand(20+33);
		writeCommand(20+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Bottom
		writeCommand(20+33);
		writeCommand(20+12);
		writeCommand(15+33);
		writeCommand(20+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWLINE); // Stem
		writeCommand(15+33);
		writeCommand(20+12);
		writeCommand(15+33);
		writeCommand(27+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);

		writeCommand(kSSD1331CommandDRAWRECT);	// Dot
		writeCommand(14+33);
		writeCommand(30+12);
		writeCommand(16+33);
		writeCommand(32+12);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0xFF);
		writeCommand(0x00);
		writeCommand(0x00);
		writeCommand(0x00);
		break;
	}

	return 0;
}

int devSSD1331bar(float certainty)
{
	writeCommand(kSSD1331CommandDRAWRECT);	// Outline
	writeCommand(35+33);
	writeCommand(10+12);
	writeCommand(40+33);
	writeCommand(30+12);
	writeCommand(0xFF);
	writeCommand(0xFF);
	writeCommand(0xFF);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x00);

	uint8_t inv_bar_height = 20 * (1-certainty);

	writeCommand(kSSD1331CommandDRAWRECT);	// Outline
	writeCommand(35+33);
	writeCommand(10+inv_bar_height+12);
	writeCommand(40+33);
	writeCommand(30+12);
	writeCommand(0xFF);
	writeCommand(0xFF);
	writeCommand(0xFF);
	writeCommand(0xFF);
	writeCommand(0xFF);
	writeCommand(0xFF);

	return 0;
}