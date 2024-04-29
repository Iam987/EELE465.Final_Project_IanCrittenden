//Master Code
// Reference: https://github.com/selimg76/nRF24-MSP430
#include <msp430.h> 
#include <math.h>
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
int RST = 0;
int err_count = 0;

//SPI Things:
int i_spi;
int j_spi;
int k_spi;
int x_spi;
int pd; //payload func index 0-15
int pd_i;  //payload func i index 0-7
int pd_x;

unsigned char status_reg;
unsigned char read_reg[5];

unsigned char read_reg_CONFIG[1];
unsigned char read_reg_EN_AA[1];
unsigned char read_reg_EN_RXADDR[1];
unsigned char read_reg_SETUP_AW[1];
unsigned char read_reg_SETUP_RETR[1];
unsigned char read_reg_RF_CH[1];
unsigned char read_reg_RF_SETUP[1];
unsigned char read_reg_STATUS[1];
unsigned char read_reg_OBSERVE_TX[1];
unsigned char read_reg_CD[1];
unsigned char read_reg_RX_ADDR_P0[5];  //5 BYTES
unsigned char read_reg_RX_ADDR_P1[5];  //5 BYTES
unsigned char read_reg_RX_ADDR_P2[1];
unsigned char read_reg_RX_ADDR_P3[1];
unsigned char read_reg_RX_ADDR_P4[1];
unsigned char read_reg_RX_ADDR_P5[1];
unsigned char read_reg_TX_ADDR[5];  //5 BYTES
unsigned char read_reg_RX_PW_P0[1];
unsigned char read_reg_RX_PW_P1[1];
unsigned char read_reg_RX_PW_P2[1];
unsigned char read_reg_RX_PW_P3[1];
unsigned char read_reg_RX_PW_P4[1];
unsigned char read_reg_RX_PW_P5[1];
unsigned char read_reg_FIFO_STATUS[1];

unsigned char clr_status[1]={0x70};

unsigned char rf_setupregister[1]={0b00000001};
unsigned char configregister[1]={0b00001110};
unsigned char rf_chanregister[1]={0b01001100};
unsigned char address[6]="00001";
unsigned char setup_retr_register[1]={0b01011111};
unsigned char en_aa_register[1]={0b00111111};
unsigned char rx_pw_register[1]={0b00100000};

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
  if(i > 65500){
      UCB0IFG |= UCSTPIFG;
      err_count += 1;
      RST = 1;
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
	Get_Temp_Array(InTemps, InTempReading);
	InTempAve = Moving_Ave(InTemps, 9);
	OutTempReading = GetTemp(OUTTEMP_Address);
    transmission_UART_buf[3] = (receive_I2C_buf[1] & BIT7) >> 7;
	Get_Temp_Array(OutTemps, OutTempReading);
	OutTempAve = Moving_Ave(OutTemps, 9);
	transmission_UART_buf[0] = OutTempAve;
	if((int)(OutTempAve * 10) - ((int)OutTempAve) * 10 >= 5){transmission_UART_buf[1] = 1;}
	else{transmission_UART_buf[1] = 0;}
	transmission_UART_buf[2] = InTempAve;
    if((int)(InTempAve * 10) - ((int)InTempAve) * 10 >= 5){transmission_UART_buf[3] = 1;}
    else{transmission_UART_buf[3] = 0;}
}
// END HANDLE_TEMPERATURES


//SPI Function Definitions
void SCLK_Pulse (void);
void Send_Bit (unsigned int value);
void CE_On (void);  //Chip enable
void CE_Off (void);  //Chip disable
void CSN_On (void);     //CSN On
void CSN_Off (void);    //CSN Off
void Write_Byte (int content);
void Instruction_Byte_MSB_First (int content);
void Read_Byte_MSB_First(int index, unsigned char regname[]);
void Write_Byte_MSB_First(unsigned char content[], int index2);
void Write_Payload_MSB_First(int pyld[], int index3);

//BEGIN INIT
void INIT(){
    __delay_cycles(100); //Power on Reset

    //NRF24L01 SPI setup
    P2OUT &= 0x00; //Clear P2OUT
    P2DIR |= MOSI + SCLK + CE + CSN; //OUTPUT pins
    P2DIR &= ~MISO; //Input pin


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
	TB1CCR0 = 512;      // 1 sec compare
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
    //UCB0CTLW0 |= UCTR; // Transmit Mode
    //rx_I2C = 0;
    //UCB0TBCNT = 1;

	//NRF stuff:

    CE_Off();
    CSN_On();
    /************************
    **CONFIGURING REGISTERS**
    *************************/
    //EN_AA  -- enabling AA in all pipes
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | EN_AA);
    Write_Byte_MSB_First(en_aa_register,1);
    CSN_On();
    //RF_SETUP
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RF_SETUP);
    Write_Byte_MSB_First(rf_setupregister,1);
    CSN_On();
    //RX_ADDR_P0
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RX_ADDR_P0);
    Write_Byte_MSB_First(address,5); // write 5 bytes address
    CSN_On();
    //TX_ADDR
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | TX_ADDR);
    Write_Byte_MSB_First(address,5); // write 5 bytes address
    CSN_On();
    //RF_CH
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RF_CH);
    Write_Byte_MSB_First(rf_chanregister,1);
    CSN_On();
    //SETUP_RETR
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | SETUP_RETR);
    Write_Byte_MSB_First(setup_retr_register,1);
    CSN_On();
    //RX_PW0
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RX_PW_P0);
    Write_Byte_MSB_First(rx_pw_register,1);
    CSN_On();
    //RX_PW1
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RX_PW_P1);
    Write_Byte_MSB_First(rx_pw_register,1);
    CSN_On();
    //RX_PW2
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RX_PW_P2);
    Write_Byte_MSB_First(rx_pw_register,1);
    CSN_On();
    //RX_PW3
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RX_PW_P3);
    Write_Byte_MSB_First(rx_pw_register,1);
    CSN_On();
    //RX_PW4
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RX_PW_P4);
    Write_Byte_MSB_First(rx_pw_register,1);
    CSN_On();
    //RX_PW4
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | RX_PW_P5);
    Write_Byte_MSB_First(rx_pw_register,1);
    CSN_On();
    //CONFIG
    CSN_Off();
    Instruction_Byte_MSB_First(W_REGISTER | CONFIG);
    Write_Byte_MSB_First(configregister,1);
    CSN_On();
    /****************************
    **END CONFIGURING REGISTERS**
    *****************************/
	__delay_cycles(2000); //Start up delay
	RST = 0;
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
		    UCB0IE &= ~UCTXIE0; // I2C Tx interrupt dissable
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
            UCB0IE |= UCTXIE0; // I2C Tx interrupt enable
		    rx_UART = 1;
		    duty = 'N';
		}
		else if(duty == 'O'){
		    GarageState = 1;
		           //STDBY-I
		           CSN_Off();
		           Instruction_Byte_MSB_First(W_TX_PAYLOAD);
		           Write_Payload_MSB_First(duty,1);
		           Write_Payload_MSB_First(duty,1);
		           CSN_On();
		           CE_On();
		           __delay_cycles(50); //min pulse >10usec
		           CE_Off();
		           //TX settling 130 usec
		           __delay_cycles(150);
		           //TX MODE

		           __delay_cycles(20000);
		           //STDBY-I
		           CSN_Off();
		           Instruction_Byte_MSB_First(NOP);
		           CSN_On();
		           if ((status_reg & BIT4) == 0x10){
		                   CSN_Off();
		                   Instruction_Byte_MSB_First(W_REGISTER | STATUS);
		                   Write_Byte_MSB_First(clr_status,1);
		                   CSN_On();
		                   CSN_Off();
		                   Instruction_Byte_MSB_First(FLUSH_TX);
		                   CSN_On();

		           }

		}
		else if(duty == 'C'){
		    GarageState = 0;
		               //STDBY-I
	                   CSN_Off();
	                   Instruction_Byte_MSB_First(W_TX_PAYLOAD);
	                   Write_Payload_MSB_First(duty,1);
	                   Write_Payload_MSB_First(duty,1);
	                   CSN_On();
	                   CE_On();
	                   __delay_cycles(50); //min pulse >10usec
	                   CE_Off();
	                   //TX settling 130 usec
	                   __delay_cycles(150);
	                   //TX MODE

	                   __delay_cycles(20000);
	                   //STDBY-I
	                   CSN_Off();
	                   Instruction_Byte_MSB_First(NOP);
	                   CSN_On();
	                   if ((status_reg & BIT4) == 0x10){
	                           CSN_Off();
	                           Instruction_Byte_MSB_First(W_REGISTER | STATUS);
	                           Write_Byte_MSB_First(clr_status,1);
	                           CSN_On();
	                           CSN_Off();
	                           Instruction_Byte_MSB_First(FLUSH_TX);
	                           CSN_On();

	                   }
		}
		else{}
		if(RST == 1){
		    INIT();
		    LM72_INIT();
		}
	}
	return 0;
}

//NRF Functions:
void SCLK_Pulse (void)
{
  P2OUT |= SCLK;//set high with OR 1
  P2OUT ^= SCLK;//toggle with XOR 1
}
void Send_Bit (unsigned int value)
{
    if (value != 0){
        P2OUT |= MOSI;}
    else {
        P2OUT &= ~MOSI;
    }
}
void CE_On (void)
{
    P2OUT |= CE;
}

void CE_Off (void)
{
    P2OUT &= ~CE;
}
void CSN_On (void)
{
    P2OUT |= CSN;
}
void CSN_Off (void)
{
    P2OUT &= ~CSN;
}
void Write_Byte(int content)  //Not ued in this application
{

    for (j_spi=0;j_spi<8;j_spi++){
             x_spi = (content & (1 << j_spi));  //Write to Address
             Send_Bit(x_spi);
             SCLK_Pulse();
        }
}
void Instruction_Byte_MSB_First(int content)
{

    for (k_spi=7;k_spi>=0;--k_spi){
             x_spi = (content & (1 << k_spi));  //Write to Address
             status_reg <<= 1;
             Send_Bit(x_spi);

             if ((P2IN & MISO) == 0x02){


                                             status_reg |= 0b00000001;
                                                }
                                                else {


                                             status_reg  &= 0b11111110;
                                                }

             SCLK_Pulse();

                         }

}
void Read_Byte_MSB_First(int index, unsigned char regname[])
{
    for (i_spi=0;i_spi<=(index-1);i_spi++){
        for (k_spi=0;k_spi<8;k_spi++){
           regname[i_spi] <<= 1;


                     if ((P2IN & MISO) == 0x02){

                                                 //read_reg |= 0b10000000;
                                                   regname[i_spi] |= 0b00000001;
                                                    }
                                                    else {

                                                 //read_reg  &= 0b01111111;
                                                   regname[i_spi]  &= 0b11111110;
                                                    }
                     SCLK_Pulse();
    }
}
}
void Write_Byte_MSB_First(unsigned char content[], int index2)
{
    for (i_spi=0;i_spi<=(index2-1);i_spi++){
    for (k_spi=7;k_spi>=0;--k_spi){

             x_spi = (content[i_spi] & (1 << k_spi));  //Write to Address
             Send_Bit(x_spi);
             SCLK_Pulse();

                         }

}
}

void Write_Payload_MSB_First(int pyld[], int index3)
{
    for (pd_i=0;pd_i<=(index3-1);pd_i++){
        for (pd=7;pd>=0;--pd){

                     pd_x = (pyld[pd_i] & (1 << pd));  //Write to Address
                     Send_Bit(pd_x);
                     SCLK_Pulse();

                                 }
        for (pd=15;pd>=8;--pd){

                             pd_x = (pyld[pd_i] & (1 << pd));  //Write to Address
                             Send_Bit(pd_x);
                             SCLK_Pulse();

                                         }

        }
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
