//Master Code
#include <msp430.h> 
#include "nrf24.h"

//Temperature sensor slave addresses
#define OUTTEMP_Address 0b1001000 //TODO A2 - A0 Ground for this
#define INTEMP_Address 0b1001100 //TODO A2 - A1 Ground A0 Vcc for this

//PIN assignments for NRF24L01
#define MOSI BIT0 // P2.0
#define MISO BIT1 // P2.1
#define SCLK BIT4 // P2.4
#define CE BIT5 // P2.5
#define CSN BIT3 // P2.3

float InTempReading;
float InTemps[9] = {9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999};
float InTempAve = 9999;
float OutTempReading;
float OutTemps[9] = {9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999, 9999};
float OutTempAve = 9999;
char GetTempFlag = 0;
unsigned int tx_I2C_index = 0, rx_I2C_index = 0, rx_UART_index = 0, tx_UART_index = 0;
char transmission_I2C_buf[32];
char receive_I2C_buf[32];
char transmission_UART_buf[32];
char receive_UART_buf[32];
int rx_I2C = 0;   // 0 means transmit mode 1 means receive mode
int rx_UART = 1;
char duty = 'N';
int GarageState = 0;

// BEGIN Get_Temp_Array
void Get_Temp_Array(float *temp_array, float temp) {
  int k;
  for (k = 8; k >= 1; k--) {
    temp_array[k] = temp_array[k - 1];
  }
  temp_array[0] = temp;
}
// END GET_TEMP_ARRAY

// BEGIN MOVING_AVE
float Moving_Ave(float *temp_array, int num) {
  float sum = 0;
  float ave;
  int i;
  for (i = 0; i < num; i++) {
    sum += temp_array[i];
  }
  ave = sum / num;
  if (temp_array[num - 1] > 9990) {
    return 9999;
  } else {
    return ave;
  }
}
// END MOVING_AVE

// BEGIN I2C_SEND
void I2C_send(int Address) {
  // Send to Slave
  UCB0I2CSA = Address;
  UCB0CTLW0 |= UCTXSTT; // Generate START condition
  unsigned int i;
  for (i = 0; i < 65530; i++) {
    if ((UCB0IFG & UCSTPIFG) != 0) {
      break;
    }
  }
  // while ((UCB0IFG & UCSTPIFG) == 0){
  // Wait for stop
  //	}
  UCB0IFG &= ~UCSTPIFG; // Clear stop flag
}
// END I2C_SEND

// BEGIN LM72_INIT
void LM72_INIT() {
	tx_I2C_index = 0;
	transmission_I2C_buf[0] = 0x01; // Pointer Byte
	transmission_I2C_buf[1] = 0x00; // Config register desired contents
	UCB0TBCNT = 2;
	I2C_send(INTEMP_Address);
	tx_I2C_index = 0;
	I2C_send(OUTTEMP_Address);
}
// END LM72_INIT

// BEGIN GETTEMP
float GetTemp(Address){
	tx_I2C_index = 0;
	transmission_I2C_buf[0] = 0x00; //Pointer Byte
	UCB0TBCNT = 1;
	I2C_send(Address);
	rx_I2C_index = 0;
	rx_I2C = 1;
	UCB0TBCNT = 2;
	UCB0CTLW0 &= ~UCTR; // Receive Mode
	UCB0IE |= UCRXIE0;  // I2C Rx interrupt enable
	I2C_send(Address);
	UCB0IE &= ~UCRXIE0; // I2C Rx interrupt disable
	rx_I2C = 0;
	UCB0CTLW0 |= UCTR; // Transmit Mode
	return receive_I2C_buf[0] + 0.5 * ((receive_I2C_buf[1] & BIT7) >> 7);
	}
// END GETTEMP

// BEGIN HANDLE_TEMPERATURES
void Handle_Temperatures() {
	GetTempFlag = 0;
	InTempReading = GetTemp(INTEMP_Address);
	transmission_UART_buf[1] = (receive_I2C_buf[1] & BIT7) >> 7;
	Get_Temp_Array(InTemps, InTempReading);
	InTempAve = Moving_Ave(InTemps, 9);
	OutTempReading = GetTemp(OUTTEMP_Address);
    transmission_UART_buf[3] = (receive_I2C_buf[1] & BIT7) >> 7;
	Get_Temp_Array(OutTemps, OutTempReading);
	OutTempAve = Moving_Ave(OutTemps, 9);
	transmission_UART_buf[0] = OutTempAve;
	transmission_UART_buf[2] = InTempAve;
}
// END HANDLE_TEMPERATURES

//BEGIN INIT
void INIT(){
	// I2C Setup
	UCB0CTLW0 |= UCSWRST;  // Software RST ON
	UCB0CTLW0 |= UCSSEL_3; // SMCLK
	UCB0BRW = 10;          // SCL 100kHz
	UCB0CTLW0 |= UCMODE_3; // I2C Mode
	UCB0CTLW0 |= UCMST;    // Master Mode
	UCB0CTLW0 |= UCTR;     // Transmit Mode
							// receiving from I2C temp sensor
	UCB0CTLW1 |= UCASTP_2; // Auto STOP when UCB0TBCNT reached
	
	UCB0CTLW0 &= ~UCSWRST; // Software RST OFF
	
	UCB0IE |= UCTXIE0; // I2C Tx interrupt enable
	
	// UART Setup
	UCA1CTLW0 |= UCSWRST; // Software RST ON
	UCA1CTLW0 |= UCSSEL__ACLK;
	UCA1BRW = 3; // BW = 9600
	UCA1MCTLW |= 0x9200;
	UCA1CTLW0 &= ~UCSWRST; //Software RST OFF
	
	//Timer setup
	TB1CTL |= TBCLR;    // Clear timer & dividers
	TB1CTL |= TBSSEL__ACLK;
	TB1EX0 |= BIT2 | BIT1 | BIT0;
	TB1CTL |= ID_3;
	TB1CTL |= MC__UP;
	TB1CCR0 = 128;      // 0.25 sec compare
	TB1CCTL0 |= CCIE;   // CCR0 interrupt enable
	TB1CCTL0 &= ~CCIFG; // Clear CCR0 flag
	
	//Port config
	P1SEL1 &= ~BIT3; // P1.3 = SCL
	P1SEL0 |= BIT3;
	
	P1SEL1 &= ~BIT2; // P1.2 = SDA
	P1SEL0 |= BIT2;
	
	P4SEL1 &= ~BIT3; // P4.3 UART TX
	P4SEL0 |= BIT3;
	
	P4SEL1 &= ~BIT2; // P4.2 UART RX
	P4SEL0 |= BIT2;
	
	// Interrupts
	UCA1IE |= UCRXIE; // UART RX interrupt enable
	// Use UCA1IE |= UCTXCPTIE; enable tx interrupt when needed
	// Use UCA1IFG &= ~UCTXCPTIFG; clear tx flag
	
	PM5CTL0 &= ~LOCKLPM5; // disable low power mode
	__enable_interrupt(); //Global interrupt enable
}
//END INIT

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	INIT();
	
	while(1){
		if (GetTempFlag == 1) {
			Handle_Temperatures();
		}

		if(duty == 'R'){
		    transmission_UART_buf[4] = GarageState;
		    tx_UART_index = 0;
		    rx_UART = 0;
		    //UCA1IE |= UCTXCPTIE; //enable tx interrupt
		    UCA1TXBUF = transmission_UART_buf[0];
		    __delay_cycles(2000);
            UCA1TXBUF = transmission_UART_buf[1];
            __delay_cycles(2000);
            UCA1TXBUF = transmission_UART_buf[2];
            __delay_cycles(2000);
            UCA1TXBUF = transmission_UART_buf[3];
            __delay_cycles(2000);
            UCA1TXBUF = transmission_UART_buf[4];
            __delay_cycles(2000);
            UCA1IE &= ~UCTXCPTIE; //Dissable tx interrupt
		    rx_UART = 1;
		    duty = 'N';
		}
		else if(duty == 'O'){
		    GarageState = 1;
		}
		else if(duty == 'C'){
		    GarageState = 0;
		}
		else{}
	}
	return 0;
}

// ISRs

// BEGIN EUSCI_B0 INTERRUPT SERVICE ROUTINE
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void) {
  if (rx_I2C == 0) {
    UCB0TXBUF = transmission_I2C_buf[tx_I2C_index];
    tx_I2C_index += 1;
  } else {
    receive_I2C_buf[rx_I2C_index] = UCB0RXBUF;
    rx_I2C_index += 1;
  }
}
// END EUSCI_B0 INTERRUPT SERVICE ROUTINE

//BEGIN EUSCI_A1 INTERRUPT SERVICE ROUTINE
#pragma vector = EUSCI_A1_VECTOR
__interrupt void EUSCI_A1_UART_ISR(void){
    if(rx_UART == 1){
        duty = UCA1RXBUF;
    }
    else{

    }
}
//END EUSCI_A1 INTERRUPT SERVICE ROUTINE

// BEGIN TIMER 1 INTERRUPT SERVICE ROUTINE (0.5s)
#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB1CCR0(void) {
  GetTempFlag = 1;
  TB1CCTL0 &= ~CCIFG; // Clear CCR0 flag
}
// END TIMER 1 INTERRUPT SERVICE ROUTINE
