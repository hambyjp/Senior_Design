/*
 * ohm_meter.c
 *
 * Created: 1/20/2016 11:40:53 PM
 *  Author: hambyje
 */ 


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stddef.h>

#define AREF	(1<<REFS0)			//uses Vcc as ref voltage (5V is maximum value to be read). write this to ADMUX
#define IN_PIN1	(1<<MUX2)			//sets PINF4 as input pin for ADC.  write this to ADMUX
#define IN_PIN2 (1<<MUX2)|(1<<MUX0) //sets PINF5 as input pin for ADC.  write this to ADMUX
#define V_REF	5					//measured with high impedance volt meter
#define CH_NUM	2					//number of resistors being measured
#define MAX_NUM 10					//max number of adc measurement samples before sending to GPRS module
#define CLOCK	16000000
#define BAUD_1	9600
//used for setting clock speed
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz       0x00
#define CPU_8MHz        0x01
#define CPU_4MHz        0x02
#define CPU_2MHz        0x03
#define CPU_1MHz        0x04
#define CPU_500kHz      0x05
#define CPU_250kHz      0x06
#define CPU_125kHz      0x07
#define CPU_62kHz       0x08
//used for light identifiers
#define LIGHT_1		0xFF
#define LIGHT_2		0x00
//if one of these bytes are received with a light identifier do appropriate action
#define CTRL_ON		0x11
#define CTRL_OFF	0x33
//if this byte is received, send a resistanced measurement
#define RES_REQUEST 0x77


/***** Configure IO *****/
//

//initialize ADC
//define in_pin that will set ADMUX register for single-ended input and pin.
//define aref to set ADMUX to use the internal reference voltage.
void init_ADC(uint8_t input_ch)
{
	ADMUX = input_ch|AREF;									//sets Vref and input pin
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//enables ADC with a clock frequency of 125kHz for 10-bit precision. 
	DIDR0 = (1<<ADC7D)|(1<<ADC6D)|(1<<ADC1D)|(1<<ADC0D);	//reduces power to unused ADC input pins: NOTE, using ADC4D and ADC5D
	DIDR2 = 0x3F;											//disables all ADC input pins in register to reduce power.
	ADMUX |= (1<<ADLAR);									//left adjusted result (i.e. read only ADCH for 8-bit precision)
}

//set up ADC Noise canceling mode
//note that ADC has to be enabled before entering noise_cancel mode
void noise_cancel_ADC()
{
	if (ADCSRA & (1<<ADEN))
	{
		while(ADCSRA & (1<<ADSC));
		SMCR |= (1<<SE);				//setting up sleep mode control register for ADC Noise Reduction Mode
		SMCR |= (1<<SM0);				//entered ADC mode, will begin conversion now.
		while(ADCSRA & (1<<ADSC));		//waiting for possible erroneous conversion to finish
	}
}

//read ADC value and return the address of the data stored
uint8_t read_ADC()
{
	uint8_t data;
	
	ADCSRA |= (1<<ADSC);		//start conversion, will use whatever input pin init_ADC() set.
	while(ADCSRA & (1<<ADSC));  //wait for conversion to complete
	data = ADCH;				//only 8-bit precision with left adjusted result (e.g. 0b_ _ _ _ _ _ _ _ X X)
								//the X X (LSB's) bits are in ADCL
	return data;
}

//function to change channels of ADC.  Verify ADC is not doing a conversion while changing inputs.
//new_input should be of form (1<<MUXx)
void change_input_ADC(uint8_t new_input)
{
	if (ADCSRA & (1<<ADSC))
	{
		while(ADCSRA & (1<<ADSC));	//waiting until conversion is complete
	}		
	ADMUX = new_input|AREF;			//changing input
}

//function to store data in flash memory
//
uint16_t store_ADC_value(uint8_t data, uint16_t address)
{
	//determine is address is within acceptable range (find out storage range from data sheet)
	
	//store in available address 
	// i.e. memory_location <-- data(8-bit)
	
	//return address where data is stored + 1
	return 0;  //place address value here.
	
}

//determine UBBR value to set baud rate for Asynchronous Mode
//function uses integer division
uint16_t baud_Calc(uint16_t baud, uint32_t clock_speed)
{
	uint32_t reg_value;
	
	//see page 191 of ATMEL datasheet for explanation
	reg_value = clock_speed/(16*(uint32_t)baud);
	reg_value = reg_value - 1;
	
	return (uint16_t)reg_value;
}


//initialize the USART communication Tx/Rx pins
//
void init_USART(uint16_t baud)
{
	//before initializing, make sure no ongoing transmissions occurring.
	//wait for empty transmit buffer
	while(!(UCSR1A & (1<<UDRE1)));
	
	//set baud rate: 9600 for GSM module
	UBRR1H = (uint8_t)(baud>>8);
	UBRR1L = (uint8_t)baud;
	
	//enabling transmiter and receiver
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	//set frame format, 8-bit data, with 2 stop bit
	UCSR1C = (1<<USBS1)|(3<<UCSZ10);	
	
	
}

//transmitt a single byte (8 bits)in ram using USART; emphasis on ram memory
//PIND3 is used for transmission (Tx pin)
void Tx_USART(uint8_t data)
{
	//wait for empty transmit buffer
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1 = data;   //send data byte
}

//transmitt AT command stored in program memory; emphasis on program memory
void Tx_USART_ATcmd(char *cmd)
{
	while(pgm_read_byte(cmd) != '\0')
	{
		Tx_USART(pgm_read_byte(cmd++));
	}
}

//transmit string in ram (in case the program memory method doesn't work)
void Tx_USART_ram_data(char *str)
{
	while(*str != '\0')
	{
		Tx_USART(*str++);
	}
}



//receive data using USART
//must accept data transmission from GSM module at 9600 baud
//Receives data serially from PIND2 (Rx pin) see data sheet for clarification
uint16_t Rx_USART()
{
	uint16_t timer = 0;
	while (!(UCSR1A & (1<<RXC1)))
	{
		timer++;
		if(timer >= 16000) return 0;
	}
	return UDR1;
	////serial in from pinD2
	////need to set frame format
	//
}


//*******************determine which light control/resistance request should reference****************




















//*******************************************************************************************************


//function for light control ON/OFF
//
/*
	code here
*/

//function for light resistance call
//
/*
	code here
*/

// so far this uses PINF4 and PINF5 for analog inputs
// PIND3 for transmit out
int main(void)
{
	//store AT command strings in program memory****************************************
	const char AT[3] PROGMEM = {"AT"};			//To initialize mode
	const char no_echo[5] PROGMEM = {"ATEo"};	//To stop echo from GSM
	const char text_mode[10] PROGMEM = {"AT+CMGF=1"}; //to gsm in text mode
	const char GSM_mode[16] PROGMEM = {"AT+CSCS=\"GSM\""}; //sets character mode: GSM 7 bit default alphabet according to (3GPP TS 23.038);
	const char parameter_set[19] PROGMEM = {"AT+CSMP=17,167,0,0"};  //sets parameter of character according to GSM protocol
	const char mobile_num[25] PROGMEM ={"AT+CMGS=\"+92090078601\""};  //sets recipient's phone number
	const char end_AT PROGMEM = 0x1A;	//character form of ctrl+z to terminate AT command
	//***********************************************************************************
	
	//store messages to be sent through GSM**********************************************
	const char mesg[12] PROGMEM = {"wow"};   //transmit this word to the receive pin  to test receive functionality.
	//***********************************************************************************
	
	//holds adc data
	uint8_t adc_data_ch1;
	uint8_t adc_data_ch2;
	uint16_t baud;
	uint8_t ch_1 = 1;
	uint8_t ch_2 = 0;
	
	//setting CPU to 16Mhz.
	CPU_PRESCALE(CPU_16MHz);		
	
	//grab readings from both sensors
	init_ADC(IN_PIN1);
	noise_cancel_ADC();
	adc_data_ch1 = read_ADC();
	change_input_ADC(IN_PIN2);
	adc_data_ch2 = read_ADC();
	
	SMCR |= (0<<SE);				//turning off noise reduction mode
	ADCSRA = (0<<ADEN);				//turning off ADC
	baud = baud_Calc(BAUD_1, CLOCK);	//returning baud word to set baud rate
	init_USART(baud);							//baud==103 for baud rate set to 9600
	
	while(1)
	{
		
		//send data through USART pin PD3
		//for testing Rx
		Tx_USART(adc_data_ch1);
		//Tx_USART(adc_data_ch2, ch_2);
		
		//test input using F_gen to input signal into recieve pin of teensy
		//for testing Rx
		//gsm_word = Rx_USART();
		//
		
	    //call function that will determine what gsm_word is telling
		//compare with stored "possible recieve commands"
		//(e.g. control for light 1 OFF/ON, light 2 OFF/ON, light 1 res_val, light 2 res_val)
		
		
		//polling for control signal to tell which channel data to send through USART to GSM
		
		//polling for control signal to tell which light is active from the control 
		//input state on the teensy for the Relays
		//(e.g. if 
		
	}
	return 0;
}
