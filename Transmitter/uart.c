#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library
#include <string.h>
#include <uart.h>

void initUART()
{
	// Configuration for 3MHz SMCLK, 9600 baud rate.
	// Calculated using the online calculator that TI provides at:
	// http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	const eUSCI_UART_Config config =
	{
		EUSCI_A_UART_CLOCKSOURCE_SMCLK, //SMCLK Clock Source
		19, //BRDIV = 19
		8, //UCxBRF = 8
		0, //UCxBRS = 0
		EUSCI_A_UART_NO_PARITY, //No Parity
		EUSCI_A_UART_LSB_FIRST, //MSB First
		EUSCI_A_UART_ONE_STOP_BIT, //One stop bit
		EUSCI_A_UART_MODE, //UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION //Oversampling
	};

	//Configure GPIO pins for UART. RX: P1.2, TX:P1.3.
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2|GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

	UART_initModule(EUSCI_A0_BASE, &config);
	UART_enableModule(EUSCI_A0_BASE);
}

//Transmit a string through UART0.
void uart0_transmitStr( const char* str )
{
	int len, i = 0;

	len = strlen( str );
	while( i < len )
	{
		UART_transmitData( EUSCI_A0_BASE, str[ i++ ] );
		while( !UART_getInterruptStatus( EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG ) );
		UART_clearInterruptFlag( EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG );
	}
}
