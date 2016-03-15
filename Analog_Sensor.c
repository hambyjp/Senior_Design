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
#include <stdio.h>

// PINB0 key pin on GSM (active low output)
// PINB1 resistor sensor enable (active low output)
// PINB2 resistor sensor enable (active low output)
// PINB7 pole 1 control (active high output)
// PINB6 pole 2 control (active high output)
// PINB3 over_voltage detect (active low input)
// PINB4 over voltage detect (active low input)
#define AREF	(1<<REFS0)			//uses Vcc as ref voltage (5V is maximum value to be read). write this to ADMUX
#define POLE1	(1<<MUX2)			//sets PINF4 as input pin for ADC.  write this to ADMUX
#define POLE2 (1<<MUX2)|(1<<MUX0)	//sets PINF5 as input pin for ADC.  write this to ADMUX
#define CTRL_1 (1<<PINB7)			
#define CTRL_2 (1<<PINB6)			
#define RES_SENS_EN1 (1<<PINB1)
#define RES_SENS_EN2 (1<<PINB2)		
#define GSM_ON	(1<<PINB0)			
#define OV_DETECT11 (1<<PINB3)		//used for masking (this is input pin)
#define OV_DETECT22 (1<<PINB4)

//UART receive byte definitions; A-I in ascii
#define LIGHT_1_CTRL_ON			0x41	//A
#define LIGHT_1_CTRL_OFF		0x42	//B	
#define LIGHT_2_CTRL_ON			0x43	//C
#define LIGHT_2_CTRL_OFF		0x44	//D
#define LIGHTS_ON				0x45	//E
#define LIGHTS_OFF				0x46	//F
#define LIGHT2_ON_LIGHT1_OFF	0x47	//G
#define LIGHT1_ON_LIGHT2_OFF	0x48	//H
#define LIGHT_1_RES_REQ			0x49	//I  if one of these bytes are received with a light identifier do appropriate action
#define LIGHT_2_RES_REQ			0x4A	//J
#define LIGHTS_RES_REQ			0x4B	//K if this byte is received, send a resistance measurement from both lights


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

const char pole_1[] = "POLE1";	//pole identifiers; send before sending data
const char pole_2[] = "POLE2";
const char poles[] = "POLES";	
const char bad_res[] = "bad";			//send before pole identifier
const char light_status[] = "status";		//send before pole identifier
const char init_status1[] = "init1";
const char init_status2[] = "init2";
const char at[] = "AT";				//to get OK response
const char text_mode[] = "AT+CMGF=1";	//to set text mode
const char full_text[] = "AT+CSDH=1";	//to display full text information
const char delete_all[] = "AT+CMGDA=\"DEL ALL\"";	//to delete all text
const char no_echo[] = "ATE0";		//turn off echo
const char reg_1[] = "AT+CMGR=1";	//to access register 1 text
const char num_cmd[] = "AT+CMGS=";	//phone number should follower this string
const char num[] = "\"15412559226\"";//phone number to send text to
const char ip_stat1[] = "67.169.210.201:3000/update1";
const char ip_stat2[] = "67.169.210.201:3000/update2";
const char ip_data1[] = "67.169.210.201:3000/data1";
const char ip_data2[] = "67.169.210.201:3000/data2";
const char ip_initstat1[] = "67.169.210.201:3000/light1";			//use on start up, don't use again
const char ip_initstat2[] = "67.169.210.201:3000/light2";			//use on strat up, don't use again
const char ip_bad_contact1[] = "67.169.210.201:3000/update1conctact";			//send a zero ascii;
const char ip_bad_contact2[] = "67.169.210.201:3000/update2conctact";			//send a zero ascii;
const char con_gprs[] = "AT+SAPBR=3,1,Contype,GPRS";
const char apn[] = "AT+SAPBR=3,1,APN,WHOLESALE";
const char en_gprs[] = "AT+SAPBR=1,1";
const char con_test[] = "AT+SAPBR=2,1";		//may not need
const char en_http[] = "AT+HTTPINIT";
const char set_profile[] = "AT+HTTPPARA=CID,1";
const char url[] = "AT+HTTPPARA=URL,";	//Tx ip_x after this
const char data_config[] = "AT+HTTPDATA=8,1500";
const char post[] = "AT+HTTPACTION=1";

const uint8_t carr_rtn = 0x0D;		//must use after every command
const uint8_t ctrl_z = 0x1A;		//after data entry when sending sms

uint8_t flag = 0;					//recieve status (when ISR(receive_vector) is processed then flag gets set)										//should reset flag in main after designed routine is processed
char data_received[1000];			//control word received from GSM--used to determine case for switch statement (state machine)
uint8_t ind = 0;					//used for indexing through array in ISR
uint32_t count = 0;					//used for the delay_functions described below
char data_ascii[8];					//used to send data in character form
uint8_t one_shot = 0;
uint8_t one_shot2 = 0;
uint8_t data_ch1=2;	//holds adc data
uint8_t data_ch2=2;	//holds adc data
char temp1[8] = {"\0"};
char temp2[8] = {"\0"};
char *data1 = temp1;
char *data2 = temp2;
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
// PINB2 resistor sensor enable (active low output)
// PINB7 pole 1 control (active high output)
// PINB6 pole 2 control (active high output)
// PINB3 over_voltage detect (active low input)
// PINB4 over voltage detect (active low input)
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/

/*****************************digital IO*****************************************/
/********************************************************************************/
void init_dio()
{
	DDRB = 0b11100111;				//1-->output, 0-->input
	PORTB = GSM_ON;									//1-->high, 0-->low	
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
	noise_cancel_ADC();
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
	ind= ind + 1;
	data_received[ind] = '\0';
}

/********************************************************************************/
/********************************************************************************/



/******************************GSM functions*************************************/
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/

/**************************Data Manipulation*************************************/
/********************************************************************************/
/********************************************************************************/
/********************************************************************************/



/******************************Binary to ASCII***********************************/
/********************************************************************************/
//data is initially pointing to first element of data array
void bin_ascii(uint8_t data, char *data_t)
{
	uint8_t temp;
	int i;
	int loc;
	for(i = 0; i<8; i++)
	{
		loc = 7-i;
		temp = data;
		temp &= (1<<i);
		if(temp)
		{
			data_t[loc] = '1';
		}
		else
		{
			data_t[loc] = '0';
		}
	}
}

/********************************************************************************/
/********************************************************************************/

/****************************************URL AT COMMANDS*************************/
/********************************************************************************/
void init_url(char *ip)
{
	Tx_USART_ram_data(con_gprs);
	Tx_USART(carr_rtn);
	delay_100m();
	Tx_USART_ram_data(apn);
	Tx_USART(carr_rtn);
	delay_100m();
	Tx_USART_ram_data(en_gprs);
	Tx_USART(carr_rtn);
	delay_100m();
	Tx_USART_ram_data(con_test);
	Tx_USART(carr_rtn);
	delay_1s();
	Tx_USART_ram_data(en_http);
	Tx_USART(carr_rtn);
	delay_100m();	
	Tx_USART_ram_data(set_profile);
	Tx_USART(carr_rtn);
	delay_100m();
	Tx_USART_ram_data(url);
	Tx_USART_ram_data(ip);
	Tx_USART(carr_rtn);
	delay_100m();
	Tx_USART_ram_data(data_config);
	Tx_USART(carr_rtn);
	delay_100m();
	//still need to input data after function call
	//still need to send a post command after data Tx.
}


/********************************************************************************/
/********************************************************************************/

/******************************Function for IP data send*************************/
/********************************************************************************/

void send_data_url(char *control_2, char *data_2)
{
	int i = 0;
	if((strcmp(control_2, pole_1)==0))
		{
			init_ADC(POLE1);
			data_ch1 = read_ADC();
			bin_ascii(data_ch1, data1);
			init_url(ip_data1);
			for (i = 0; i <8; i++)
			{
				Tx_USART(data1[i]);
			}
			delay_2s();
			Tx_USART_ram_data(post);
			Tx_USART(carr_rtn);
			delay_1s();
		}
	
	if((strcmp(control_2, pole_2)==0))
	{
		init_ADC(POLE2);
		data_ch2 = read_ADC();
		bin_ascii(data_ch2, data2);
		init_url(ip_data2);
		for (i = 0; i <8; i++)
		{
			Tx_USART(data2[i]);
		}
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
	}

	if((strcmp(control_2, poles)==0))
	{
		init_ADC(POLE1);
		data_ch1 = read_ADC();
		bin_ascii(data_ch1, data1);
		init_ADC(POLE2);
		data_ch2 = read_ADC();
		bin_ascii(data_ch2, data2);
		init_url(ip_data2);
		for (i = 0; i <8; i++)
		{
			Tx_USART(data2[i]);
		}
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
		init_url(ip_data2);
		for (i = 0; i <8; i++)
		{
			Tx_USART(data2[i]);
		}
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
		
	}
	
	if((strcmp(control_2, bad_res)==0)&&(strcmp(data_2, pole_1)==0))
	{
		init_url(ip_bad_contact1);
		Tx_USART_ram_data("0");
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
		
	}
	if((strcmp(control_2, bad_res)==0)&&(strcmp(data_2, pole_2)==0))
	{
		init_url(ip_bad_contact2);
		Tx_USART_ram_data("0");
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
	}
	
	if((strcmp(control_2, light_status)==0)&&(strcmp(data_2, pole_1)==0))
	{
		init_url(ip_stat1);
		Tx_USART_ram_data("true");
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
	}	
	
	if((strcmp(control_2, light_status)==0)&&(strcmp(data_2, pole_2)==0))
	{
		init_url(ip_stat2);
		Tx_USART_ram_data("true");
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
	}
	
	if((strcmp(control_2, light_status)==0)&&(strcmp(data_2,"1OFF")== 0))
	{
		init_url(ip_stat1);
		Tx_USART_ram_data("false");
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
	}
	
	if((strcmp(control_2, light_status)==0)&&(strcmp(data_2, "2OFF")== 0))
	{
		init_url(ip_stat2);
		Tx_USART_ram_data("false");
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
	}
	
	
	if((strcmp(control_2, init_status1)==0))
	{
		init_url(ip_initstat1);
		Tx_USART_ram_data("true");
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
	}
	
	if((strcmp(control_2, init_status2)==0))
	{
		init_url(ip_initstat2);
		Tx_USART_ram_data("true");
		delay_2s();
		Tx_USART_ram_data(post);
		Tx_USART(carr_rtn);
		delay_1s();
	}
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
	//delay_10m();
	Tx_USART(carr_rtn);
	delay_2s();
	//strcpy(data_received, "\0");
	ind = 0;
}


/********************************************************************************/
/********************************************************************************/


/***************************send data through sms *******************************/
/********************************************************************************/
//be sure to set text mode before use
void send_data_sms(char *message)
{
	Tx_USART_ram_data(num_cmd);
	Tx_USART_ram_data(num);
	Tx_USART(carr_rtn);
	delay_10m();
	Tx_USART_ram_data(message);
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
	//strcpy(data_received, "\0");	//must use for get_ctrl()'s strrchr() ref.
	Tx_USART_ram_data(reg_1); //accessing register 1
	Tx_USART(carr_rtn);
	delay_2s();
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


/********************************pin outs for teensy*****************************/
// PINF4 pole 1 resistor (analog input) 
// PINF5 pole 2 resistor (analog input)
// PIND3 Tx	(USART)
// PIND2 Rx	(USART)
// PINB0 key pin on GSM (active low output)
// PINB1 resistor sensor enable (active low output)
// PINB2 resistor sensor enable (active low output)
// PINB7 pole 1 control (active high output)
// PINB6 pole 2 control (active high output)
// PINB3 over_voltage detect (active low input)
// PINB4 over voltage detect (active low input)
/********************************************************************************/

int main(void)
{
	//uint8_t data_ch1;	//holds adc data
	//uint8_t data_ch2;	//holds adc data
	char cmd_reg = '\0';	//register where sms is
	char cmd_word = '\0';	//data in text (a command word)
	char *cmd;				//command text
	uint8_t point;
	int i = 0;				//used for looping
	uint8_t ov_detect1;
	uint8_t ov_detect2;
	//char temp1[8] = {"\0"};
	//char temp2[8] = {"\0"};
	//char *data1 = temp1;
	//char *data2 = temp2;
	//setting CPU to 16Mhz.
	CPU_PRESCALE(CPU_16MHz);
	//initialize I/O		
	init_dio();
	//initialize ADC
	init_ADC(POLE1);					//pinf4
	//initialize USART
	init_USART(BAUD);					//baud==103 for baud rate set to 9600

	sei();								//ready to receive interrupts
	
	////***************TESTING ADC TO CHARACTER****************************************//
	//while (1)
	//{
		//init_ADC(POLE1);
		//data_ch1 = read_ADC();
		//bin_ascii(data_ch1, data1);
		//init_ADC(POLE2);
		//data_ch2 = read_ADC();
		//bin_ascii(data_ch2, data2);
		//for (i = 0; i <8; i++)
		//{
			//Tx_USART(data1[i]);
		//}
			//Tx_USART_ram_data(data1);
		//for (i = 0; i <8; i++)
			//{
			//Tx_USART(data2[i]);
			//}
	//}
	////******************************************************************************//
	
	
	
	
	Tx_USART(carr_rtn);					//incase of intermittent commands sent before
	
	delay_2s();
	delay_2s();
	ind = 0;
	echo_off();
	ind = 0;
	delay_100m();
	
	
	if (!pwr_chkGSM())					//checking if GSM is on, if not it will turn it back on
	{
		on_gsm();	
		Tx_USART(carr_rtn);
		delay_2s();
		ind = 0;
		echo_off();				
		//ind = 0;
	}
	echo_off();							//for some unknown reason... :(
	
	set_Textmode();
	delete_sms();						//delete any commands received while off
	ind = 0;							//in case text notifications received
	
	send_data_url(init_status1, "true");
	delete_sms();
	ind = 0;
	send_data_url(init_status2, "true");
	delete_sms();
	ind = 0;
	/**************************TESTING TEXT RECEIVE*****************************************/
	//**************************PASSED!!!!!!!!!!!!!*****************************************/
	//testing receiving of text messages
	//while(1)
	//{
		//Tx_USART(ind);
		//if(ind > 13)
		//{
			//delay_2s();
			//cmd_reg = data_received[14];  //location in sms received reply string with data register location
			//if(cmd_reg == '1')			  //#include <string.h> and <stdio.h>... dope!
				//{
					//read_SMS();
					//ind = 0;
					//cmd = get_ctrl();
					//cmd_word = *cmd;
					//if(cmd_word > 64 && cmd_word < 74)
					//{
							//switch (cmd_word)
							//{
								//case LIGHT_1_CTRL_OFF:
									//PORTB &= ~CTRL_1;
									//send_data_url(ip, light_status, "1 OFF");
									//delete_sms();
									//ind = 0;
									//break;
								//case LIGHT_1_CTRL_ON:
									//PORTB |= CTRL_1;
									//send_data_url(ip, light_status, "1 ON");
									//delete_sms();
									//ind = 0;
								//break;
								//default:
									//while(1)
									//{
										//delay_1s();
										//Tx_USART(0xFF);
									//}
								//break;
							//}
					//}
				//}
			//
//
		//}
	//}
	/*****************************************************************************************/
	
	/**********************************for testing**********************************************/
	//////testing for ok response from gsm.. size should be 6 and data_received should be {CR,LF,O,K,CR,LF}
	////
	//
	//uint8_t size = strlen(data_received);
	//while(1){
		//delay_1s();
		//Tx_USART(size);				 //should be 0b0011000001 on OSCOPE
		//Tx_USART(data_received[2]);  //should be 0b0111100101 on OSCOPE
	//}  //testing incrementally
	/*********************************************************************************************/
	
	//SHOULD SEND INITIAL STATUS OF LIGHTS TO IP ADDRESS
	//
	
	//
	//send status of light before sending data
	/*********************************************/
	/***********code goes here********************/
	//
	//****************MUST MUST MUST MUST HAVE HAVE HAVE HAVE***********************************/
	//ov_detect1 = PINB;
	//ov_detect2 = PINB;
	//ov_detect1 &= OV_DETECT1;			//masking out the over voltage detect
	//ov_detect2 &= OV_DETECT2;
	//*************MUSTHAVE************************************///
	//sending 10 data samples for each resistance
	for (i=0; i<10; i++)
	{
		//if (!ov_detect1)
		//{
			//PORTB |= RES_SENS_EN1;		//setting resistor enable high-->OFF
			//if(!one_shot)
			//{
				//send_data_sms(bad_res, pole_1);
				//send_data_sms();
				//delete_sms();
				//ind = 0;
				//one_shot = 1;
			//}
		//}
		//if (!ov_detect2)
		//{
			//PORTB |= RES_SENS_EN2;
			//if(!one_shot2)
			//{
				//send_data_sms(bad_res, pole_2);
				//delete_sms();
				//ind = 0;
				//one_shot2 = 1;
			//}
		//}
		change_input_ADC(POLE1);
		data_ch1 = read_ADC();
		//change_input_ADC(POLE2);		//pinf5
		//data_ch2 = read_ADC();
		bin_ascii(data_ch1, data1);	//converting to ascii string 0b"xxxxxxxx"
		////data2 = bin_ascii(data_ch2);
		send_data_url(pole_1, data1);
		delete_sms();
		ind = 0;
		//send_data_url(pole_2, data2);
		//delete_sms();
		//ind = 0;
	}
	//****************************************************************///
	//
	
	//*********************************MAIN STATE MACHINE***************************************//
	while(1)
	{
		ov_detect1 = PINB;
		//ov_detect2 = PINB;
		ov_detect1 &= OV_DETECT11;			//masking out the over voltage detect
		//ov_detect2 &= OV_DETECT22;
		if (!ov_detect1)
		{
		
			if(!one_shot)
			{
					PORTB |= RES_SENS_EN1;		//setting resistor enable high-->OFF
				//send_data_url(bad_res, pole_1);
				//ind = 0; 
				//send_data_sms("POLE 1: OPEN CONTACT");
				//delete_sms();
				//ind = 0;
				//one_shot = 1;
			}
		}
		//if (!ov_detect2)
		//{
			//
			//if(!one_shot2)
			//{
				//PORTB |= RES_SENS_EN2;
				////send_data_url(bad_res, pole_2);
				//ind = 0;
				////send_data_sms("POLE 2: OPEN CONTACT");
				//delete_sms();
				//ind = 0;
				//one_shot2 = 1;
			//}
		//}	
		//
		if(ind > 13)
		{
			cmd_reg = data_received[14];  //location in sms received reply string with data register location
			if(cmd_reg == '1')
			{
				read_SMS();
				ind = 0;
				cmd = get_ctrl();
				cmd_word = *cmd;
				if(cmd_word > 64 && cmd_word < 76)  //A through I capitols matter!!!
				{
					delete_sms();
					delay_2s();
					switch (cmd_word)
					{
						case LIGHT_1_CTRL_ON:
							PORTB &= ~CTRL_1;
							send_data_url(light_status, pole_1);
							delete_sms();
							ind = 0;
						break;
						case LIGHT_1_CTRL_OFF:
							PORTB |= CTRL_1;
							send_data_url(light_status, "1OFF");
							delete_sms();
							ind = 0;
						break;
						case LIGHT_2_CTRL_ON:
							PORTB &= ~CTRL_2;
							send_data_url(light_status, pole_2);
							delete_sms();
							ind = 0;
						break;
						case LIGHT_2_CTRL_OFF:
							PORTB |= CTRL_2;
							send_data_url(light_status, "2OFF");
							delete_sms();
							ind = 0;
						break;
						case LIGHTS_ON:
							PORTB &= ~CTRL_1;
							PORTB &= ~CTRL_2;
							send_data_url(light_status, pole_1);
							delete_sms();
							ind = 0;
							
							send_data_url(light_status, pole_2);
							delete_sms();
							ind = 0;
						break;
						case LIGHTS_OFF:
							PORTB |= CTRL_1;
							PORTB |= CTRL_2;
							send_data_url(light_status, "1OFF");
							delete_sms();
							ind = 0;
							
							send_data_url(light_status, "2OFF");
							delete_sms();
							ind = 0;
						break;
						case LIGHT2_ON_LIGHT1_OFF:
							PORTB |= CTRL_1;
							PORTB &= ~CTRL_2;
							send_data_url(light_status, "1OFF");
							delete_sms();
							ind = 0;
							send_data_url(light_status, pole_2);
							delete_sms();
							ind = 0;
						break;
						case LIGHT1_ON_LIGHT2_OFF:
							PORTB &= ~CTRL_1;
							PORTB |= CTRL_2;
							send_data_url(light_status, pole_1);
							delete_sms();
							ind = 0;
							send_data_url(light_status, "2OFF");
							delete_sms();
							ind = 0;
						break;
						case LIGHT_1_RES_REQ:
							change_input_ADC(POLE1);
							data_ch1 = read_ADC();
							//bin_ascii(data_ch1, data1);
							send_data_url(pole_1, data1);
							delete_sms();
							ind = 0;
						break;
						case LIGHT_2_RES_REQ:
							change_input_ADC(POLE2);
							data_ch2 = read_ADC();
							//bin_ascii(data_ch2, data2);
							send_data_url(pole_2, data2);
							delete_sms();
							ind = 0;
						break;
						case LIGHTS_RES_REQ:
							change_input_ADC(POLE1);
							data_ch1 = read_ADC();
							//bin_ascii(data_ch1, data1);
							send_data_url(pole_1, data1);
							delete_sms();
							ind = 0;
							//change_input_ADC(POLE2);
							//data_ch2 = read_ADC();
							////bin_ascii(data_ch2, data2);
							//send_data_url(pole_2, data2);
							//delete_sms();
							//ind = 0;
						break;
						default:
							send_data_sms("NOT WORKING");
							delete_sms();
							ind = 0;
						break;
					}
				}
			}
		}
	}
	return 0;
}
