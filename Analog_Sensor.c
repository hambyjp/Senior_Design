/*
 * Analog_Sensor.c
 *
 * Created: 1/20/2016 11:40:53 PM
 *  Author: hambyje
 */ 
#define F_CPU			0x00		//16MHz
//source code dependencies
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include <avr/interrupt.h>


#define AREF	(1<<REFS0)			//uses Vcc as ref voltage (5V is maximum value to be read). write this to ADMUX
#define IN_PIN1	(1<<MUX2)			//sets PINF4 as input pin for ADC.  write this to ADMUX
#define IN_PIN2 (1<<MUX2)|(1<<MUX0) //sets PINF5 as input pin for ADC.  write this to ADMUX
#define CTRL_PIN0 (1<<PINB3)		//output
#define CTRL_PIN1 (1<<PINB4)		//output
#define RES_SENS_EN (1<<PINC7)		//output
#define GSM_ON	(1<<PINB0)			//output


//UART receive byte definitions
#define LIGHT_1_CTRL_ON	 0xF1
#define LIGHT_1_CTRL_OFF 0xF0
#define LIGHT_2_CTRL_ON  0x1F
#define LIGHT_2_CTRL_OFF 0x0F
#define LIGHTS_ON		 0xFF
#define LIGHTS_OFF		 0x11
#define LIGHT_1_RES_REQ	 0xF7	//if one of these bytes are received with a light identifier do appropriate action
#define LIGHT_2_RES_REQ  0x7F
#define LIGHTS_RES_REQ   0x77	//if this byte is received, send a resistance measurement from both lights


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
#define BAUD	103					//baud rate of 9600: determined from data sheet

uint8_t flag = 0x00;				//recieve status (when ISR(receive_vector) is processed then flag gets set)
									//should reset flag in main after designed routine is processed
uint8_t data_received;				//control word received from GSM--used to determine case for switch statement (state machine)

uint32_t count = 0;					//used for the delay_functions described below


/************************delay_functions*****************************************/
/********************************************************************************/


void delay_1s()
{
	while (count < 1350000)
	{
		count++;
	}
	count = 0;
}

void delay_2s()
{
	while (count < 2700000)
	{
		count++;
	}
	count= 0;
}

void delay_500m()
{
	while (count < 670000)
	{
		count++;
	}
	count=0;
}

void delay_100m()
{
	while (count < 135000)
	{
		count++;
	}
	count = 0;
}

void delay_10m()
{
	while (count < 15920)
	{
		count++;
	}
	count = 0;
}


/********************************************************************************/
/********************************************************************************/



/*****************************Configure IO **************************************/
/********************************************************************************/

//sensor enable dependent on sensor input from comparator
void init_output_enable()
{
	DDRB = 0xFF;	//all portB pins are now outputs
	PORTB = 0x00;	//all portB pins are low state (~0v)	
}

//initialize input for comparator
void init_ov_detect()
{
	DDRC = 0x00;	//all port C pins are now inputs
	PORTC = 0x00;  //high Z tri-state (won't source current when pulled low :)
}
/********************************************************************************/
/********************************************************************************/



/*****************************ADC Configuration**********************************/
/********************************************************************************/

//initialize ADC
//define in_pin that will set ADMUX register for single-ended input and pin.
//define aref to set ADMUX to use the internal reference voltage.
//input pins are either pinf4--ch1 or pinF5--ch2
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

//read ADC value and return the address of the data stored either pinF4 or pin F5
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
//either pinF4 or pinF5
void change_input_ADC(uint8_t new_input)
{
	if (ADCSRA & (1<<ADSC))
	{
		while(ADCSRA & (1<<ADSC));	//waiting until conversion is complete
	}		
	ADMUX = new_input|AREF;			//changing input
}
/********************************************************************************/
/********************************************************************************/



/****************************UART Configuration**********************************/
/********************************************************************************/

//initialize Tx/Rx pins
void init_USART(uint16_t baud)
{
	//before initializing, make sure no ongoing transmissions occurring.
	//wait for empty transmit buffer
	while(!(UCSR1A & (1<<UDRE1)));
	
	//set baud rate: 9600 for GSM module
	UBRR1H = (uint8_t)(baud>>8);
	UBRR1L = (uint8_t)baud;
	
	//enabling transmiter and receiver
	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);
	//set frame format, 8-bit data, with 1 stop bit
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);	
		
}

//transmitt a single byte (8 bits)in ram using USART; emphasis on ram memory
//PIND3 is used for transmission (Tx pin)
void Tx_USART(uint8_t data)
{
	//wait for empty transmit buffer
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1 = data;   //send data byte
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
uint8_t Rx_USART()
{
	uint16_t timer = 0;
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	while (!(UCSR1A & (1<<RXC1)))
	{
		timer++;
		if(timer >= 16000) return 0;
	}
	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);
	return UDR1;
	////serial in from pinD2
	////need to set frame format
	//
}


/*************************Configuration of Rx Interrupts*************************/
/********************************************************************************/



/*******************************Receive Interrupt********************************/
/********************************************************************************/

ISR(USART1_RX_vect)
{
	//PORTB ^= (1<<PINB0);
	if(!(UCSR1A & (1<<RXC1)))
	data_received = UDR1;
	delay_1s();
	flag = 0xFF;
}

/********************************************************************************/
/********************************************************************************/

/******************************GSM functions*************************************/


/********************************Function for SMS call***************************/
/********************************************************************************/

void send_sms(char *at, uint8_t carr_rtn, char *no_echo, char *init_sms, char *phone_num, char *msg, uint8_t ctrl_z){
	delay_2s();
	Tx_USART_ram_data(at);
	Tx_USART(carr_rtn);
	delay_2s();
	Tx_USART_ram_data(no_echo);
	Tx_USART(carr_rtn);
	delay_2s();
	Tx_USART_ram_data(init_sms);
	Tx_USART(carr_rtn);
	delay_2s();
	Tx_USART_ram_data(phone_num);
	Tx_USART(carr_rtn);
	delay_2s();
	Tx_USART_ram_data(msg);
	delay_2s();
	Tx_USART(ctrl_z);
	delay_2s();
}

/********************************************************************************/
/********************************************************************************/

/******************************Function for IP data send*************************/
/********************************************************************************/

void send_data(){}

/********************************************************************************/
/********************************************************************************/

/******************************TURNING ON GSM ***********************************/
/********************************************************************************/

void on_gsm(){
	//turn on gsm module
	PORTB &= ~GSM_ON;
	delay_2s();
	delay_2s();				//requires a 2sec low on key pin to turn on
	delay_500m();
	PORTB |= GSM_ON;
}

/********************************************************************************/
/********************************************************************************/

// so far this uses PINF4 and PINF5 for analog inputs
// PIND3 for transmit out
int main(void)
{
	//holds adc data
	uint8_t adc_data_ch1;
	uint8_t adc_data_ch2;
	uint8_t ch_1 = 1;
	uint8_t ch_2 = 0;
	char at[3] = {"AT"};		//use to auto configure baud rate (9600)
	uint8_t carr_rtn = 0x0D;  //must use after every command
	char init_sms[10] = {"AT+CMGF=1"};
	char phone_num[22] = {"AT+CMGS=\"15415252700\""};
	char msg[15] = {"what's up ash?"};
	char no_echo[5] = {"ATE0"};
	uint8_t ctrl_z = 0x1A;
	
	//int i = 0;				//used for testing
	
	//setting CPU to 16Mhz.
	CPU_PRESCALE(CPU_16MHz);		
	
	//grab readings from both sensors
	//use to test ADC
	init_ADC(IN_PIN1);  //pinf4
	noise_cancel_ADC();
	
	//change_input_ADC(IN_PIN2);
	//adc_data_ch2 = read_ADC();
	
	//SMCR |= (0<<SE);					//turning off noise reduction mode
	//ADCSRA = (0<<ADEN);					//turning off ADC
	init_output_enable();
	init_USART(BAUD);					//baud==103 for baud rate set to 9600
	//sei();
	delay_10m();						//wait 10ms then turn on gsm module (pull Key pin high)
	//on_gsm();							//turn on gsm module	
	//send_sms(at, carr_rtn, no_echo, init_sms, phone_num, msg, ctrl_z);
	//on_gsm();							//turn off gsm module
	
	while(1)
	{
		
		
		//TESTING RECEIVE INTERRUPT ROUTINE FUNCTIONALITY
		//tie PINB1 to Rx PIND2
		//while (i < 8){
		//PORTB ^= (1<<PINB1);
		//_delay_loop_1(0xFF);
		//_delay_loop_1(0xFF);
		//_delay_loop_1(0xFF);
		//i++;
		//}
		//i = 0;
		//
		//testing if ISR for receive vector was ran  will toggle if so (when PINB1(output) is tied to PIND2(Rx))
		//if(flag == 0xFF)
		//{
			//PORTB ^= CTRL_PIN1;
			//flag = 0x00;
		//}
		
		
		//TESTING ADC FUNCTIONALITY
		adc_data_ch1 = read_ADC();
		//used to test if transmitting the correct voltage and ADC value
		Tx_USART(adc_data_ch1);
		
		
		
		/*********system level************state maching****************************/
		//if(flag == 0xFF)
		//{
			//
			////switch (data_received)
			////{
				////case LIGHT_1_CTRL_OFF:
				//////
				////break;
				////case LIGHT_1_CTRL_ON:
				//////
				////break;
				////case LIGHT_2_CTRL_OFF:
				//////
				////break;
				////case LIGHT_2_CTRL_ON:
				//////
				////break;
				////case LIGHTS_OFF:
				//////
				////break;
				////case LIGHTS_ON:
				//////
				////break;
				////case LIGHTS_RES_REQ:
				//////
				////break;
				////default:
				////ERROR MESSAGE SENT TO WEB APPLICATION FOR BAD CONTROL WORD OR WORD NOT PROCESSED
				////break;
			////}
			//flag = 0x00;
		//}
	}
	return 0;
}
