/*
* SensorInterfacing.c
*
* Created: 27-Sep-19 9:04:13 AM
* Author: Shivam Mahesh Potdar (github/shivampotdar) (shivampotdar99@gmail.com)
* This code forms an interface for MQ2 gas sensor with Firebird, values of LPG, CO and Smoke detected in PPM are
* shown on the 16x2 LCD onboard.
*
* The MQ-2 gas sensor has three outputs pins, connections can be made as follows :
* - The ATMEGA2560 Microcontroller Board expansion socket on Firebird can be used for the purpose:
* - Note that all four jumpers from J2 should be removed to make use of the ADC pins on the socket
* - This would disable IR Proximity sensor 1-4
* - DO pin on the MQ2 module can be left unused as it is used with a comparator on-board just to detect threshold
*
* MQ-2 <---------> Firebird
*  Vcc             Pin21/22 (5V)
*  GND				Pin23/24 (Ground)
*  AO				Pin7-10 (7=ADC6,8=ADC7,9=ADC5,10=ADC4)
*
* References :
* 1. MQ2 Datasheet - https://www.pololu.com/file/0J309/MQ2.pdf
* 2. https://www.instructables.com/id/How-to-Detect-Concentration-of-Gas-by-Using-MQ2-Se/
*/

#define F_CPU 14746500

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>									//included to support power function
#include "lcd.h"									//This file has been created newly as including .c was giving linker errors, the functions in lcd.c are also corrected for some errors.

#define MQ_PIN 4									//ADC Channel number on where the sensor analog

int RL_VALUE=5;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in data sheet


const int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
const int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time internal(in millisecond) between each samples in the calibration phase
const int READ_SAMPLE_INTERVAL=50;                        //define how many samples you are going to take in normal operation
const int READ_SAMPLE_TIMES=5;                            //define the time internal(in millisecond) between each samples in normal operation

#define         GAS_LPG             0
#define         GAS_CO              1
#define         GAS_SMOKE           2

float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms



/****************** gasReading ****************************************
Input:   channel, ADC channel to which MQ2 sensor AO pin is connected
Output:  calculated ADC value (0-255)
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
unsigned int gasReading(unsigned char channel)
{
	unsigned int gasValue = 0;
	channel = channel & 0x07;
	ADMUX= 0x20 | channel;								    // 0x20 is for ADLAR (left adjust) and then | Ch for setting ADMUX(4:0) bits
	ADCSRA = ADCSRA | 0x40;									// Set start conversion bit
	while((ADCSRA&0x10)==0);								// Wait for conversion to complete
	gasValue=ADCH;
	ADCSRA = ADCSRA|0x10;									// clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return gasValue;
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int channel)
{
  unsigned int raw_adc = gasReading(channel);				//read ADC value
  return ( ((float)RL_VALUE*(255-raw_adc)/raw_adc));		//Convert reading to resistance
}


/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int channel)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(channel);
    _delay_ms(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                           //according to the chart in the data sheet 

}

/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to calculate the sensor resistance (Rs).
         The Rs changes as the sensor is in the different concentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int channel)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(channel);
    _delay_ms(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}


/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
long MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
  return 0;
}
 

void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7;									//all the LCD pin's direction set as output
	PORTC = PORTC & 0x08;								// all the LCD pins are set to logic 0 except PORTC 7
}

void gas_sensor_port_config (void)
{
	DDRF = 0x00;										//All ADC pins set as input and initialised to 0
	PORTF = 0x00;
	ADCSRA = 0x00;										// disable ADC during initialization.
	ADCSRB = 0x00;										// Write suitable value in this register for initialization.
	ADMUX  = 0x20;										// select external Reference voltage (connected to AREF pin) and left adjustment result (LAR) active.
	ACSR   = 0x80;										// Disable the analog comparator
	ADCSRA = 0x86;										// enable ADC and select pre-scalar as 64
}

void port_init()
{
	lcd_port_config();
	gas_sensor_port_config();
}

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	lcd_init();
	sei(); //Enables the global interrupts
}


int main(void)
{
	init_devices();
	lcd_string(1,1,"Calibrating...");
	Ro = MQCalibration(MQ_PIN);
	lcd_clear();
	lcd_string(1,1,"Done!..");
	lcd_string(2,1,"Ro= ");
	lcd_print(2,5,Ro,6);
	lcd_string(2,12,"kOhm");
	_delay_ms(3000);
	//This is just an example listed below. You can update each of the variables however you want	
    while(1)
    {
		//Call function and print them on the LCD
		//Use the following function to print out your sensor reading
		
		long iPPM_LPG = 0;
		long iPPM_CO = 0;
		long iPPM_Smoke = 0;

		iPPM_LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
		iPPM_CO = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
		iPPM_Smoke = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
		
		lcd_clear();
		lcd_string(1,1,"LPG=");
		lcd_print(1,5,iPPM_LPG,4);
		lcd_string(1,9,"CO=");
		lcd_print(1,12,iPPM_CO,4);
		lcd_string(2,1,"Smoke=");
		lcd_print(2,7,iPPM_Smoke,4);
		
		_delay_ms(100);
    }
}