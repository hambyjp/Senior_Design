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
#include <string.h>
#include <stdlib.h>

// PINB0 key pin on GSM (active low output)
// PINB1 resistor sensor enable (active low output)
// PINB7 pole 1 control (active high output)
// PINB6 pole 2 control (active high output)
// PINB3 over_voltage detect (active low input)
#define AREF	(1<<REFS0)			//uses Vcc as ref voltage (5V is maximum value to be read). write this to ADMUX
#define POLE1	(1<<MUX2)			//sets PINF4 as input pin for ADC.  write this to ADMUX
#define POLE2 (1<<MUX2)|(1<<MUX0)	//sets PINF5 as input pin for ADC.  write this to ADMUX
#define CTRL_1 (1<<PINB7)			
#define CTRL_2 (1<<PINB6)			
#define RES_SENS_EN (1<<PINB1)		
#define GSM_ON	(1<<PINB0)			
#define OV_DETECT (1<<PINB3)		//used for masking (this is input pin)


//UART receive byte definitions; A-I in ascii
#define LIGHT_1_CTRL_ON	 0x41
#define LIGHT_1_CTRL_OFF 0x42
#define LIGHT_2_CTRL_ON  0x43
#define LIGHT_2_CTRL_OFF 0x44
#define LIGHTS_ON		 0x45
#define LIGHTS_OFF		 0x46
#define LIGHT_1_RES_REQ	 0x47	//if one of these bytes are received with a light identifier do appropriate action
#define LIGHT_2_RES_REQ  0x48
#define LIGHTS_RES_REQ   0x49	//if this byte is received, send a resistance measurement from both lights


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

const char pole_1[] = "00000000";	//pole identifiers; send before sending data
const char pole_2[] = "00000001";	
const char bad_res[] = "bad";			//send before pole identifier
const char light_status[] = "status";		//send before pole identifier
const char at[] = "AT";				//to get OK response
const char text_mode[] = "AT+CMGF=1";	//to set text mode
const char full_text[] = "AT+CSDH=1";	//to display full text information
const char delete_all[] = "AT+CMGDA=\"DEL ALL\"";	//to delete all text
const char no_echo[] = "ATE0";		//turn off echo
const char reg_1[] = "AT+CMGR=1";	//to access register 1 text
const char num_cmd[] = "AT+CMGS=";	//phone number should follower this string
const char num[] = "\"15412559226\"";//phone number to send text to
const char ip[] = "";

const uint8_t carr_rtn = 0x0D;		//must use after every command
const uint8_t ctrl_z = 0x1A;		//after data entry when sending sms

uint8_t flag = 0;					//recieve status (when ISR(receive_vector) is processed then flag gets set)										//should reset flag in main after designed routine is processed
char data_received[100];			//control word received from GSM--used to determine case for switch statement (state machine)
uint8_t ind = 0;					//used for indexing through array in ISR
uint32_t count = 0;					//used for the delay_functions described below
char data_ascii[8];					//used to send data in character form


/************************delay_functions*****************************************/
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/
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
/********************************************************************************/
/********************************pin outs for teensy*****************************/
// PINB0 key pin on GSM (active low output)
// PINB1 resistor sensor enable (active low output)
// PINB7 pole 1 control (active high output)
// PINB6 pole 2 control (active high output)
// PINB3 over_voltage detect (active low input)
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/

/*****************************digital IO*****************************************/
/********************************************************************************/
void init_dio()
{
	DDRB = ~(1<<PINB3);				//1-->output, 0-->input
	PORTB = (1<<PINB0)|(1<<PINB1);	//1-->high, 0-->low	
}

/********************************************************************************/
/********************************************************************************/



/*****************************ADC Configuration**********************************/
/********************************************************************************/
/********************************pin outs for teensy*****************************/
// PINF4 pole 1 resistor (analog input)
// PINF5 pole 2 resistor (analog input)
/********************************************************************************/
/********************************************************************************/
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
/********************************pin outs for teensy*****************************/
// PIND3 Tx	(USART)
// PIND2 Rx	(USART)
/********************************************************************************/
/********************************************************************************/
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
	
	//enabling transmiter and receiver(interrupt based)
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
	//UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	while (!(UCSR1A & (1<<RXC1)))
	{
		timer++;
		if(timer >= 16000) return 0;
	}
	//UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);
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
	data_received[ind] = UDR1;
	ind++;
}

/********************************************************************************/
/********************************************************************************/



/******************************GSM functions*************************************/
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/


/******************************Function for IP data send*************************/
/********************************************************************************/

void send_data_url(char *ip_2, char *control_2, char *data_2)
{
	
}


/********************************************************************************/
/********************************************************************************/

/******************************** turn on GSM ***********************************/
/********************************************************************************/

void on_gsm(){
	//turn on gsm module
	PORTB &= ~GSM_ON;
	delay_2s();			//requires a 2sec low on key pin to turn on
	delay_500m();
	PORTB |= GSM_ON;
	delay_2s();			//wait long time for power on notifications and sms notifications
	delay_2s();
	delay_2s();
	delay_2s();	
	ind = 0;			//reset after notifications
}

/********************************************************************************/
/********************************************************************************/

/**************************** turn off echo GSM *********************************/
/********************************************************************************/
void echo_off()
{
	Tx_USART_ram_data(no_echo);			//turn off echo
	delay_10m();
	Tx_USART(carr_rtn);
	delay_500m();
	//strcpy(data_received, "\0");
	ind = 0;
}


/********************************************************************************/
/********************************************************************************/


/***************************send data through sms *******************************/
/********************************************************************************/
//be sure to set text mode before use
void send_data_sms(uint8_t data)
{
	Tx_USART_ram_data(num_cmd);
	Tx_USART_ram_data(num);
	Tx_USART(carr_rtn);
	delay_10m();
	Tx_USART(data);
	delay_10m();
	Tx_USART(ctrl_z);
	delay_1s();
	ind = 0;
}

/********************************************************************************/
/********************************************************************************/

/********************************* set Text mode ********************************/
/********************************************************************************/
void set_Textmode()
{
	ind = 0;
	Tx_USART_ram_data(text_mode);
	Tx_USART(carr_rtn);
	delay_100m();
	ind = 0;
	Tx_USART_ram_data(full_text);
	Tx_USART(carr_rtn);
	delay_100m();
}
/********************************************************************************/
/********************************************************************************/

/******************************power check GSM***********************************/
/********************************************************************************/
//returns 1 if power on; returns 0 if not powered on
uint8_t pwr_chkGSM()
{
	ind = 0;
	Tx_USART_ram_data(at);
	Tx_USART(carr_rtn);
	delay_1s();			//wait for receive to complete
	if(data_received[2]=='O')	
	{
		ind = 0;
		return 1;
	}
	return 0; //TEE HEE >:( -- Bear was here!
}

/********************************************************************************/
/********************************************************************************/

/******************************Delete all SMS************************************/
/********************************************************************************/
//be in text mode before calling this
void delete_sms()
{	
	ind = 0;
	Tx_USART_ram_data(delete_all);
	Tx_USART(carr_rtn);
	delay_2s();		//max response time
	delay_2s();
}
/********************************************************************************/
/********************************************************************************/

/***********************************read SMS*************************************/
/********************************************************************************/
//be sure to delete all messages initially (when unit powers on)
//this means there won't be any saved commands.
//also be sure to run set_textmode as well
void read_SMS()
{
	ind = 0;
	strcpy(data_received, "\0");	//must use for get_ctrl()'s strrchr() ref.
	Tx_USART_ram_data(reg_1); //accessing register 1
	Tx_USART(carr_rtn);
}

/********************************************************************************/
/********************************************************************************/

/*************************get control from SMS***********************************/
/********************************************************************************/
//run directly after read_sms to get a ctrl word.
char* get_ctrl()
{
	char *place;
	place = strrchr(data_received, ',');  //be sure to set text mode and show all text data!
	place = place + 4;					  //now pointing at cmd_word if GSM set in text mode!
	return place;
}
/********************************************************************************/
/********************************************************************************/

/**************************Data Manipulation*************************************/
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/



/******************************Binary to ASCII***********************************/
/********************************************************************************/
//data is initially pointing to first element of data array
char * bin_ascii(uint8_t data)
{
	uint8_t temp;
	int i;
	for(i = 0; i<8; i++)
	{
		temp = data;	
		temp &= (1<<i);
		if(temp)
			data_ascii[i] = '1';
		data_ascii[i] = '0';
		temp++;
	}
	return data_ascii;
}

/********************************************************************************/
/********************************************************************************/


/********************************pin outs for teensy*****************************/
// PINF4 pole 1 resistor (analog input) 
// PINF5 pole 2 resistor (analog input)
// PIND3 Tx	(USART)
// PIND2 Rx	(USART)
// PINB0 key pin on GSM (active low output)
// PINB1 resistor sensor enable (active low output)
// PINB7 pole 1 control (active high output)
// PINB6 pole 2 control (active high output)
// PINB3 over_voltage detect (active low input)
/********************************************************************************/

int main(void)
{
	uint8_t data_ch1;	//holds adc data
	uint8_t data_ch2;	//holds adc data
	char cmd_reg = '\0';	//register where sms is
	char cmd_word = '\0';	//data in text (a command word)
	char *cmd;				//command text
	uint8_t point;
	int i = 0;				//used for looping
	uint8_t ov_detect;
	char *data1;
	char *data2;
	//setting CPU to 16Mhz.
	CPU_PRESCALE(CPU_16MHz);
	//initialize I/O		
	init_dio();
	//initialize ADC
	init_ADC(POLE1);					//pinf4
	noise_cancel_ADC();
	
	
	//SMCR |= (0<<SE);					//turning off noise reduction mode maybe??
	strcpy(data_received, "\0");        //initial data buffer set to null
	init_USART(BAUD);					//baud==103 for baud rate set to 9600
	sei();								//ready to receive interrupts
	
	if (!pwr_chkGSM())					//checking if GSM is on, if not it will turn it back on
	{
		on_gsm();						
		echo_off();
		ind = 0;
	}
	else
		echo_off();						//keeps gsm from echoing every character also reset data_received
	
	set_Textmode();
	delete_sms();						//delete any commands received while off
	ind = 0;							//in case text notifications received
	
	//sending 10 data samples for each resistance
	for (i=0; i<10; i++)
	{
		change_input_ADC(POLE1);
		ov_detect = PINB;
		ov_detect &= OV_DETECT;			//masking out the over voltage detect
		if (ov_detect)
		{
			PORTB |= RES_SENS_EN;		//setting resistor enable high-->OFF
			send_data_url(ip, bad_res, pole_1);
		}
		data_ch1 = read_ADC();
		change_input_ADC(POLE2);		//pinf5
		data_ch2 = read_ADC();
		
		data1 = bin_ascii(data_ch1);	//converting to ascii string 0b"xxxxxxxx"
		data2 = bin_ascii(data_ch2);
		send_data_url(ip, pole_1, data1);
		send_data_url(ip, pole_2, data2);
	}

									
	//testing for ok response from gsm.. size should be 6 and data_received should be {CR,LF,O,K,CR,LF}
	//uint8_t size = strlen(data_received);
	//while(1){
		//delay_1s();
		//Tx_USART(size);				 //should be 0b0011000001 on OSCOPE
		//Tx_USART(data_received[2]);  //should be 0b0111100101 on OSCOPE
	//}  //testing incrementally
	
	while(1)
	{	
		ov_detect = PINB;
		ov_detect &= OV_DETECT;			//masking out the over voltage detect
		if (ov_detect)
		{
			PORTB |= RES_SENS_EN;		//setting resistor enable high-->OFF
			send_data_url(ip, bad_res, pole_1);
		}
		
		if(ind > 14 && ind < 20)
		{
			cmd_reg = data_received[12];  //location in sms received reply string with data register location
			if(cmd_reg == '1')
			{
				read_SMS();
				ind = 0;
				cmd = get_ctrl();
				cmd_word = *cmd;
				if(cmd_word > 64 && cmd_word < 74)
				{
					delete_sms();
					switch (cmd_word)
					{
						case LIGHT_1_CTRL_OFF:
							PORTB &= ~CTRL_1;
							send_data_url(ip, light_status, "1 OFF");
							ind = 0;
						break;
						case LIGHT_1_CTRL_ON:
							PORTB |= CTRL_1;
							send_data_url(ip, light_status, "1 ON");
							ind = 0;
						break;
						case LIGHT_2_CTRL_OFF:
							PORTB &= ~CTRL_2;
							send_data_url(ip, light_status, "2 OFF");
							ind = 0;
						break;
						case LIGHT_2_CTRL_ON:
							PORTB |= CTRL_2;
							send_data_url(ip, light_status, "2 ON");
							ind = 0;
						break;
						case LIGHTS_OFF:
							PORTB &= ~CTRL_1;
							send_data_url(ip, light_status, "1 OFF");
							ind = 0;
							PORTB &= ~CTRL_2;
							send_data_url(ip, light_status, "2 OFF");
							ind = 0;
						break;
						case LIGHTS_ON:
							PORTB |= CTRL_1;
							send_data_url(ip, light_status, "1 ON");
							ind = 0;
							PORTB |= CTRL_2;
							send_data_url(ip, light_status, "2 ON");
							ind = 0;
						break;
						case LIGHT_1_RES_REQ:
							change_input_ADC(POLE1);
							data_ch1 = read_ADC();
							data1 = bin_ascii(data_ch1);
							send_data_url(ip, pole_1, data1);
							ind = 0;
						break;
						case LIGHT_2_RES_REQ:
							change_input_ADC(POLE2);
							data_ch2 = read_ADC();
							data2 = bin_ascii(data_ch2);
							send_data_url(ip, pole_2, data2);
							ind = 0;
						break;
						case LIGHTS_RES_REQ:
							change_input_ADC(POLE1);
							data_ch1 = read_ADC();
							data1 = bin_ascii(data_ch1);
							send_data_url(ip, pole_1, data1);
							ind = 0;
							change_input_ADC(POLE2);
							data_ch2 = read_ADC();
							data2 = bin_ascii(data_ch2);
							send_data_url(ip, pole_2, data2);
							ind = 0;
						break;
						default:
							Tx_USART_ram_data("SOS");
						break;
					}
				}
			}
		}
	}
	return 0;
}
