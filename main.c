#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>  //Exact-width integer types
#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <motor.h>
#include <tachometer.h>
#include <bumpSensors.h>
#include <uart.h>

#define CLOCK_HF		48000000 	// 48MHz
#define CLOCK_LF		32000 		// 32kHz

#define HEARTBEAT_FREQ	2  			// unit: Hz, heart beat LED blinking frequency for debugging

#define RED_LED 		GPIO_PIN0  // heart beat LED
#define GREEN_LED		GPIO_PIN1
#define BLUE_LED		GPIO_PIN2

#define LEFT_MOTOR		0
#define RIGHT_MOTOR		1

void initDevice_HFXT();
void initHeartBeatLED();

uint32_t clockMCLK, clockSMCLK;
uint8_t currentLED = RED_LED;

const char *terminalDisplayText = 	"\r\nTI-RSLK MAX Motor Control Demo\r\n"
									"  C: Change LED Color, L: Left Motor, R: Right Motor, S: Change Direction\r\n"
									"  I: Increase DutyCycle, D: Decrease DutyCycle, V: Display Speed, H: Help\r\n"
									"> ";

void main(void)
{
	char str[ 100 ];
	uint8_t data;

	initDevice_HFXT();
	initHeartBeatLED();
	initUART();
	initMotors( clockSMCLK );
	initBumpSensors( clockMCLK );
	initTachometers( clockSMCLK );

	Interrupt_enableMaster();
	Timer32_startTimer(TIMER32_0_BASE, false);

	// Start timer for tachometer speed measurements
	startTacho();

	//Initial display on terminal.
	uart0_transmitStr( terminalDisplayText );

	while(1)
	{
		if(UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG))
		{
			data = UART_receiveData(EUSCI_A0_BASE);
			UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

			switch(data)
			{
				case 'C':
				case 'c':
					currentLED = (currentLED == BLUE_LED) ? RED_LED : (currentLED << 1); //cycle through three colors
					uart0_transmitStr("Change blinking LED color.\r\n> ");
					break;

				case 'H':
				case 'h':
					uart0_transmitStr( terminalDisplayText );
					break;

				case 'S':
				case 's':
					switchDirection( LEFT_MOTOR );
					switchDirection( RIGHT_MOTOR );

					sprintf( str, "Change motor direction: %i, %i.\r\n> ", getMotorDirection( LEFT_MOTOR ), getMotorDirection( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;

				case 'I':
				case 'i':
					increasePWM( LEFT_MOTOR );
					increasePWM( RIGHT_MOTOR );

					sprintf( str, "Increase PWM duty cycle: %i, %i.\r\n> ", getMotorDutyCycle( LEFT_MOTOR ), getMotorDutyCycle( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;

				case 'D':
				case 'd':
					decreasePWM( LEFT_MOTOR );
					decreasePWM( RIGHT_MOTOR );

					sprintf( str, "Decrease PWM duty cycle: %i, %i.\r\n> ", getMotorDutyCycle( LEFT_MOTOR ), getMotorDutyCycle( RIGHT_MOTOR ) );
					uart0_transmitStr( str );
					break;
				case 'V':
				case 'v':
					sprintf( str, "Left RPS = %.1f\r\nLeft Direction = %d\r\nRight RPS = %.1f\r\nRight Direction = %d\r\n\n",
							 getSpeed( LEFT_MOTOR ) / 60, getTachoDirection( LEFT_MOTOR ),
							 getSpeed( RIGHT_MOTOR ) / 60, getTachoDirection( RIGHT_MOTOR ) );
					uart0_transmitStr(str);
					break;
			} //end of switch
		} //end of if

		// Calls when a bump button is pressed.
		if( bumpStateSet() )
		{
			// Check if bump button is still pressed
			int bumpState = checkBumpState();
			if( bumpState == 0 )
			{
				uart0_transmitStr( "Button released.\r\n\n" );

				// Restart motors
				startMotor( LEFT_MOTOR );
				startMotor( RIGHT_MOTOR );
			}
			else
			{
				char str[ 100 ];
				sprintf( str, "Bump triggered: %i\r\n", bumpState );
				uart0_transmitStr( str );

				// Stop motors while bump sensor is active
				pauseMotor( LEFT_MOTOR );
				pauseMotor( RIGHT_MOTOR );
			}
		}
	} //end of while
}

void initDevice_HFXT(void)
{
	WDT_A_holdTimer();  //Stop watchdog timer.

	//Change VCORE to 1 to support a frequency higher than 24MHz.
	//See MSP432 Data Sheet, Section 5.8 for Flash wait-state requirements for active frequency.
	PCM_setPowerState(PCM_AM_DCDC_VCORE1);
	FlashCtl_setWaitState(FLASH_BANK0, 1);
	FlashCtl_setWaitState(FLASH_BANK1, 1);

	FPU_enableModule();
	FPU_enableLazyStacking(); //Required to use FPU within ISR.

	//Configure PJ.2 and PJ.3 in HFXT mode.
	//Initialize external clock sources HFXT.
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN2|GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
	CS_setExternalClockSourceFrequency(CLOCK_LF, CLOCK_HF);
	CS_startHFXT(false);

	CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
	CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_8);
	CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);

	clockMCLK = CS_getMCLK();
	clockSMCLK = CS_getSMCLK();
}

void initHeartBeatLED(void)
{
	//Configure P2.0, P2.1, P2.2 as output.
	//P2.0, P2.1, P2.2 are connected to a RGB tri-color LED on LaunchPad.
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);

	Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
	Timer32_setCount(TIMER32_0_BASE, clockMCLK/HEARTBEAT_FREQ);
	Timer32_enableInterrupt(TIMER32_0_BASE);
	Interrupt_enableInterrupt(INT_T32_INT1); // Enable Timer32_0 interrupt in the interrupt controller.
}

//Timer32_0 ISR
void T32_INT1_IRQHandler(void)
{
	Timer32_clearInterruptFlag(TIMER32_0_BASE);

	if(GPIO_getInputPinValue(GPIO_PORT_P2, RED_LED|GREEN_LED|BLUE_LED))
		GPIO_setOutputLowOnPin(GPIO_PORT_P2, RED_LED|GREEN_LED|BLUE_LED);
	else
		GPIO_setOutputHighOnPin(GPIO_PORT_P2, currentLED);
}
