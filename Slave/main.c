//Garage Slave Code
// Reference: https://github.com/selimg76/nRF24-MSP430/blob/main/receiver_main.c

#include <msp430.h> 
#include "nrf24.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//PIN assignments for NRF24L01
#define MOSI BIT0 // P1.0
#define MISO BIT1 // P1.1
#define SCLK BIT4 // P1.4
#define CE BIT5 // P1.5
#define CSN BIT3 // P1.3

int i;
int j;
int k;
int l; //for pload2, so many indexes..
int x;
int pd; //payload func index 0-15
int pd_i;  //payload func i index 0-7
int pd_x;
int pipe_nr;
int pyld1[8]; //1st 16 bytes in rx fifo
int pyld2[8]; //2nd 16 bytes in rx fifo

unsigned char status_reg;
unsigned char read_reg[5];
char buf[5];
char pipe_nr_chr[5];
char bosluk[] = " ";
char next_satir[] = "\r\n";  //sorry for mixing eng and tur. i'm doing it on purpose :)

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
unsigned char read_PAYLOAD[32];

unsigned char clr_status[1]={0x70}; //clr status reg

unsigned char rf_setupregister[1]={0b00000001};  //Data Rate -> '0' (1 Mbps) & PA_MIN
unsigned char configregister[1]={0b00001111};  //CRC '1'-> (2 bytes)  & Power ON & Enable CRC & RECEIVER -------1
unsigned char rf_chanregister[1]={0b01001100};  //Channel '1001100'
unsigned char address[6]="00001";  //write to RX_ADDR_P0 and TX_ADDR
unsigned char setup_retr_register[1]={0b01011111};  //retry values
unsigned char en_aa_register[1]={0b00111111};
unsigned char rx_pw_register[1]={0b00100000};  //RX_ payload width register -->32

void SCLK_Pulse (void);  //To create a clock pulse high low
void Send_Bit (unsigned int value);     //For sending 1 or zero
void CE_On (void);  //Chip enable
void CE_Off (void);  //Chip disable
void CSN_On (void);     //CSN On
void CSN_Off (void);    //CSN Off
void Write_Byte (int content);
void Instruction_Byte_MSB_First (int content);
void Read_Byte_MSB_First(int index, unsigned char regname[]);
void Write_Byte_MSB_First(unsigned char content[], int index2);
void Write_Payload_MSB_First(int pyld[], int index3);
void ser_output(char *str);  //Serial output func

