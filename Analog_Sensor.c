/*
 * Analog_Sensor.c
 *
 * Created: 1/20/2016 11:40:53 PM
 *  Author: hambyje
 */ 
 
 

//libraries from Atmel Studio
//set up for ATmega32U4 processor

#include <avr/io.h>
#include <avr/pgmspace.h>

#define aref	(1<<REFS0)			//uses Vcc as ref voltage (5V is maximum value to be read). write this to ADMUX
#define in_pin1	(1<<MUX2)			//sets PINF4 as input pin for ADC.  write this to ADMUX
#define in_pin2 (1<<MUX2)|(1<<MUX0) //sets PINF5 as input pin for ADC.  write this to ADMUX
#define V_ref	5					//measured with high impedance volt meter
#define ch_num	2					//number of resistors being measured
#define max_num 10					//max number of adc measurement samples before sending to GPRS module
#define clock	16000000
#define baud_1	9600
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

/***** Configure IO *****/
//

//initialize ADC
//define in_pin that will set ADMUX register for single-ended input and pin.
//define aref to set ADMUX to use the internal reference voltage.
void init_ADC(uint8_t input_ch)
{
	ADMUX = input_ch|aref;									//sets Vref and input pin
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
	ADMUX = new_input|aref;			//changing input
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

//initialize the USART communication Tx/Rx pins
//
void init_USART(uint16_t baud)
{
	//before initializing, make sure no ongoing transmissions occuring.
	//wait for empty transmit buffer
	while(!(UCSR1A & (1<<UDRE1)));
	
	//set baud rate: 9600 for GSM module I think...
	UBRR1H = (uint8_t)(baud>>8);
	UBRR1L = (uint8_t)baud;
	
	//enabling transmitt or receiver
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	//set frame format, 8-bit data, with 2 stop bit
	UCSR1C = (1<<USBS1)|(3<<UCSZ10);	
	
	//before initializing, make sure no ongoing transmissions occuring.
	//wait for empty transmit buffer
	while(!(UCSR1A & (1<<UDRE1)));
	
	//set baud rate: 9600 for GSM module I think...
	UBRR1H = (uint8_t)(baud>>8);
	UBRR1L = (uint8_t)baud;
	
	//enabling transmitt or receiver
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	//set frame format, 8-bit data, with 2 stop bit
	UCSR1C = (1<<USBS1)|(3<<UCSZ10);
}

//transmitt data using USART
//
void Tx_USART(uint8_t data)
{
	//wait for empty transmit buffer
	while(!(UCSR1A & (1<<UDRE1)));
	
	UDR1 = data;
}

//determine UBBR value to set baud rate for Asynchronous Mode
//function uses integer division
uint16_t baud_Calc(uint16_t baud, uint32_t clock_speed)
{
	uint16_t reg_value;
	
	//see page 191 of ATMEL datasheet for explanation
	reg_value = clock_speed/(16*baud);
	reg_value = reg_value - 1;
	
	return reg_value; 
}


// so far this uses PINF4 and PINF5 for analog inputs
// PIND3 for transmit out
int main(void)
{
	uint8_t adc_data_ch1;
	uint8_t adc_data_ch2;
	uint16_t baud_value;
	baud_value = baud_1;
	uint16_t baud_word;
	uint32_t clock_spd;
	clock_spd = clock;
	uint8_t test;
	test = 0b10101010;
	CPU_PRESCALE(CPU_16MHz);		//setting CPU to 16Mhz.
	
	//grab readings from both sensors
	init_ADC(in_pin1);
	noise_cancel_ADC();
	adc_data_ch1 = read_ADC();
	//change_input_ADC(in_pin2);
	//adc_data_ch2 = read_ADC();
	//SMCR |= (0<<SE);				//turning off noise reduction mode
	//ADCSRA = (0<<ADEN);				//turning off ADC
	baud_word = baud_Calc(baud_value, clock_spd);	//returning baud word to set baud rate
	init_USART(baud_word);							//setting things
	//Tx_USART(0b11001010);
	//Tx_USART(0b10101010);
	
	while(1)
	{
		adc_data_ch1 = read_ADC();
		Tx_USART(adc_data_ch1);
		//Tx_USART(0b10000000);
		//send data through USART pin PD3
		//Tx_USART(0b11110000);
		//Tx_USART(adc_data_ch2);
	}
	return 0;
}
