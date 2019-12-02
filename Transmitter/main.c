#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>  //Exact-width integer types
#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <uart.h>

#define CLOCK_HF        48000000    // Hz
#define CLOCK_LF        32000       // Hz
#define SAMPLE_FREQ     100        // Hz. ADC can't take more than 24 MHz per sample. (P6.0 and P6.1 are each 1 sample). 50 Hz per sample is enough I think

#define HEARTBEAT_FREQ  2           // unit: Hz, heart beat LED blinking frequency for debugging

#define RED_LED         GPIO_PIN0  // heart beat LED
#define GREEN_LED       GPIO_PIN1
#define BLUE_LED        GPIO_PIN2

#define LEFT_MOTOR		0
#define RIGHT_MOTOR		1
#define MOTOR_FORWARD	1
#define MOTOR_STOP		0
#define MOTOR_BACKWARD	-1

#define UPPER_BOUNDARY	70
#define LOWER_BOUNDARY	30

void initDevice_HFXT();
void initADC14();
void initHeartBeatLED();
void initTimer();

uint32_t clockMCLK, clockSMCLK, clockACLK;
uint8_t currentLED = RED_LED;

const char *terminalDisplayText =   "\r\nTI-RSLK MAX Motor Control Demo\r\n"
                                    "  C: Change LED Color, S: Change Direction, I: Increase DutyCycle,\r\n"
                                    "  D: Decrease DutyCycle, V: Display Speed, H: Help\r\n"
                                    "> \r\n";

void main(void)
{
    uint8_t data;

    initDevice_HFXT();
    initADC14();
    initHeartBeatLED();
    initUART();
    initTimer();

    Interrupt_enableMaster();
    Timer32_startTimer( TIMER32_0_BASE, false );

    //Initial display on terminal.
    uart0_transmitStr( terminalDisplayText );
    Timer32_startTimer( TIMER32_1_BASE, false );

    while(1)
    {
        if(UART_getInterruptStatus( EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG ) )
        {
            data = UART_receiveData( EUSCI_A0_BASE);
            UART_clearInterruptFlag( EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG );

            switch(data)
            {
                case 'H':
                case 'h':
                    uart0_transmitStr( terminalDisplayText );
                    break;
            } //end of switch
        } //end of if
    } //end of while
}

void initDevice_HFXT()
{
    WDT_A_holdTimer();

    PCM_setPowerState( PCM_AM_DCDC_VCORE1 );
    FlashCtl_setWaitState( FLASH_BANK0, 1 );
    FlashCtl_setWaitState( FLASH_BANK1, 1 );

    FPU_enableModule();
    FPU_enableLazyStacking();

    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_PJ, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION );
    CS_setExternalClockSourceFrequency( CLOCK_LF, CLOCK_HF );
    CS_startHFXT(false);

    CS_initClockSignal( CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_8 );
    CS_initClockSignal( CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16 );

    clockMCLK = CS_getMCLK();
    clockSMCLK = CS_getSMCLK();
}

void initADC14()
{
    ADC14_enableModule();
    ADC14_initModule( ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE );

	//Configure P6.1 as A14 and P6.0 as A15
	GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION );

	// Configure non-repeating multi-sequence memory storage in registers 0 and 1
	ADC14_configureMultiSequenceMode( ADC_MEM0, ADC_MEM1, false );

	// Store P6.1 (A14) reading in MEM0 and P6.0 (A15) reading in MEM1
	ADC14_configureConversionMemory( ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false );
	ADC14_configureConversionMemory( ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false );

    //See TechRef Section 20.2.6 for sample timing consideration.
    ADC14_enableSampleTimer( ADC_MANUAL_ITERATION );
    ADC14_setSampleHoldTime( ADC_PULSE_WIDTH_4, ADC_PULSE_WIDTH_4 );

    ADC14_enableConversion();
}

void initHeartBeatLED()
{
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 );
    GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 );
    GPIO_setAsInputPin( GPIO_PORT_P3, GPIO_PIN2 );
}

void initTimer()
{
    Timer32_initModule( TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE );
    Timer32_setCount( TIMER32_0_BASE, clockMCLK / HEARTBEAT_FREQ );
    Timer32_enableInterrupt( TIMER32_0_BASE );
    Interrupt_enableInterrupt( INT_T32_INT1 );

    Timer32_initModule( TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE );
    Timer32_setCount( TIMER32_1_BASE, clockMCLK / SAMPLE_FREQ );
    Timer32_enableInterrupt( TIMER32_1_BASE );
    Interrupt_enableInterrupt( INT_T32_INT2 );
}

void T32_INT1_IRQHandler()
{
    Timer32_clearInterruptFlag( TIMER32_0_BASE );

	if(GPIO_getInputPinValue( GPIO_PORT_P2, RED_LED | GREEN_LED | BLUE_LED ) )
		GPIO_setOutputLowOnPin( GPIO_PORT_P2, RED_LED | GREEN_LED | BLUE_LED );
	else
		GPIO_setOutputHighOnPin( GPIO_PORT_P2, currentLED );
}

void T32_INT2_IRQHandler()
{
    Timer32_clearInterruptFlag( TIMER32_1_BASE );

	// Throttle on P6.1 (A14, MEM0)
	// Steering on P6.0 (A15, MEM1)
	uint16_t dataInt[ 2 ] = { 0 };
	float dutyCycle, steering;

	//Read from ADC.
	ADC14_toggleConversionTrigger(); //ADC14SC is reset automatically

	// Get data from ADC registers
	ADC14_getMultiSequenceResult( dataInt );

	// [0, 100]
	dutyCycle = 100 * ( float ) dataInt[ 0 ] / 0x3FFF;
	steering = 100 * ( float ) dataInt[ 1 ] / 0x3FFF;

	// Reset everything. data0 | data1 | data2 | data3
	GPIO_setOutputHighOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 );

	if( steering > UPPER_BOUNDARY )
	{
		// Turn Right
		// Check if forward, reverse, or neither
		if( dutyCycle > UPPER_BOUNDARY )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN3 );				// 1101
		else if( dutyCycle < LOWER_BOUNDARY )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN4 );				// 1110
		else
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN3 | GPIO_PIN4 );	// 1100
	}
	else if( steering < LOWER_BOUNDARY )
	{
		// Turn Right
		// Check if forward, reverse, or neither
		if( dutyCycle > UPPER_BOUNDARY )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN2 | GPIO_PIN3 );				// 1001
		else if( dutyCycle < LOWER_BOUNDARY )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN2 | GPIO_PIN4 );				// 1010
		else
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 );	// 1000
	}
	else if( dutyCycle > UPPER_BOUNDARY )
	{
		if( dutyCycle < UPPER_BOUNDARY + ( ( 100 - UPPER_BOUNDARY ) / 4 ) )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 );	// 0000
		else if( dutyCycle < UPPER_BOUNDARY + ( ( 100 - UPPER_BOUNDARY ) / 2 ) )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 );				// 0001
		else if( dutyCycle < UPPER_BOUNDARY + ( 3 * ( 100 - UPPER_BOUNDARY ) / 4 ) )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN4 );				// 0010
		else
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 );							// 0011
	}
	else if( dutyCycle < LOWER_BOUNDARY )
	{
		if( dutyCycle > ( 3 * ( LOWER_BOUNDARY / 4 ) ) )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN3 | GPIO_PIN4 );				// 0100
		else if( dutyCycle > ( LOWER_BOUNDARY / 2 ) )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN3 );							// 0101
		else if( dutyCycle > ( LOWER_BOUNDARY / 4 ) )
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN4 );							// 0110
		else
			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 );										// 0111

//		if( dutyCycle < ( LOWER_BOUNDARY / 4 ) )
//			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN3 | GPIO_PIN4 );				// 0100
//		else if( dutyCycle < ( LOWER_BOUNDARY / 2 ) )
//			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN3 );							// 0101
//		else if( dutyCycle < ( 3 * ( LOWER_BOUNDARY / 4 ) ) )
//			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN4 );							// 0110
//		else
//			GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 );										// 0111
	}
	else
	{
		// Set all data pins high
		GPIO_setOutputHighOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 );
	}
}
