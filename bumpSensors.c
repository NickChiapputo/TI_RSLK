#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <bumpSensors.h>

#define BUMP0    			GPIO_PIN0  	// Right side of robot.
#define BUMP1    			GPIO_PIN2
#define BUMP2    			GPIO_PIN3
#define BUMP3    			GPIO_PIN5
#define BUMP4    			GPIO_PIN6
#define BUMP5    			GPIO_PIN7  	// Left side of robot.
#define NUM_DEBOUNCE_CHECKS 10			// For 50 ms debounce time.
#define SYSTICK_FREQ		200			// Frequency of SysTick timer in Hz.

volatile uint_fast16_t bumpStatus = 0;	// Used to determine which button is being pressed
volatile uint_fast16_t bumpPress = 0;	// Represents whether a button is being pressed. Is only 0 or 1
volatile uint32_t bumpButton;			// Last pressed button
volatile uint32_t buttonStateIndex;		// Used to control debouncing. Counts number of times SysTick_Handler is called. Max value is NUM_DEBOUNCE_CHECKS
int clockMCLK;							// Frequency of the MCLK

void initBumpSensors( int clockMCLK )
{
	clockMCLK = clockMCLK;

	GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P4, BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5 );

	// Set any of the bump buttons to trigger interrupt on falling edge (when pressed)
	GPIO_enableInterrupt( GPIO_PORT_P4, BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5 );
	GPIO_interruptEdgeSelect( GPIO_PORT_P4, BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5, GPIO_HIGH_TO_LOW_TRANSITION );

	// Set the SysTick period so that debouncing for the bump sensors can be executed
	SysTick_setPeriod( clockMCLK / SYSTICK_FREQ - 1 );

	Interrupt_enableInterrupt( INT_PORT4 );
}

int bumpStateSet()
{
	return bumpPress;
}

int checkBumpState()
{
	uint8_t bumpState = GPIO_getInputPinValue( GPIO_PORT_P4, bumpButton );
	if( bumpState )
	{
		bumpPress = 0;
		return 0;
	}

	return ( bumpStatus & BUMP0 ) ? 1 :
			( bumpStatus & BUMP1 ) ? 2 :
			( bumpStatus & BUMP2 ) ? 3 :
			( bumpStatus & BUMP3 ) ? 4 :
			( bumpStatus & BUMP4 ) ? 5 : 6 ;
}

void PORT4_IRQHandler()
{
	bumpStatus = GPIO_getEnabledInterruptStatus( GPIO_PORT_P4 );
	GPIO_clearInterruptFlag( GPIO_PORT_P4, bumpStatus );

	// Determine which button is pressed
	bumpButton = ( bumpStatus & BUMP0 ) ? BUMP0 :
				( bumpStatus & BUMP1 ) ? BUMP1 :
				( bumpStatus & BUMP2 ) ? BUMP2 :
				( bumpStatus & BUMP3 ) ? BUMP3 :
				( bumpStatus & BUMP4 ) ? BUMP4 : BUMP5;

	// Start SysTick for debouncing the button press(es)
	buttonStateIndex = 0;
	SysTick->VAL = 0;
	SysTick_enableModule();
	SysTick_enableInterrupt();
}

void SysTick_Handler()
{
	// Since buttons are configured with pull-up resistor, the pin value is 0 when pressed
	uint8_t buttonState = GPIO_getInputPinValue( GPIO_PORT_P4, bumpButton );
	buttonState = bumpButton & ( ~buttonState );
	buttonStateIndex++;

	if( ( buttonState & bumpButton ) == 0 )
	{
		SysTick_disableInterrupt();
		SysTick_disableModule();
	}
	else if( buttonStateIndex >= NUM_DEBOUNCE_CHECKS )
	{
		// Button is confirmed pressed
		SysTick_disableInterrupt();
		SysTick_disableModule();

		bumpPress = 1;
	}
}
