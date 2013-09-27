/*
Test/Sample code for using an MMA7660FC Accelerometer.
From https://github.com/brodykenrick/MMA7660.
Written by Brody Kenrick July 2013
Released into the public domain.

I have used this module that contains voltage regulation:
http://dx.com/p/mma7660fc-3-axial-triaxial-digital-accelerometer-module-blue-179445

Testing on an Arduino Uno
Circuit
MMA7660Board.INT to Pin 8 (Digital 8)
MMA7660Board.SDA to SDA
MMA7660Board.SCL to SCL
MMA7660Board.VCC to VCC
MMA7660Board.GND to GND

The test program will provide various outputs to the Serial based on polling the unit and interrupts from tapping, shaking or rotating the unit.
*/


//Include a TWI/I2C interface for Arduino to link against.
//The actual selection is made inside the library header file.
#if defined(MMA7660_USE_NB_I2C)
//I2C from NinjaBlocks @ Github: https://github.com/ninjablocks/arduino/tree/master/I2C
#include <I2C.h>
#elif defined(MMA7660_USE_ARDUINO_WIRE)
#include <Wire.h>
#elif defined(MMA7660_USE_SOFT_TWI)
#include <SoftTWI.h> //In this (MMA7660) github repository (sourced from http://code.google.com/p/fivevolt/source/browse/Quadcopter/Arduino/Telemetry/SoftTWI.h/.cpp)
#endif
//NOTE: The above '#if's don't really remove the headers/libs from the project because of the way the Arduino IDE determines libraries. Comment it out see the real effect on size.

#include <MMA7660.h>
#include <PinChangeInt.h> //Hardware External Interrupt Pins run out and the pinchangeint library is used to get a few more usable interrupts

const int accelInterruptPin =  8;

MMA7660 g_Accel = MMA7660();

//Interrupt function connected to pin accelInterruptPin
volatile unsigned char isr_accel_flag = 0;
void isr_accel() {
  if(PCintPort::arduinoPin == accelInterruptPin) {
    isr_accel_flag++;
  }
}


//This is the "filtered callback" function called from the MMA7660 library.
//If you want to get some filtered detections and nicely enumerated types of the interrupt use something like
// this (in conjunction with filterProcessTime and filterProcessInterrupt)
//If you want to process it yourself then you should do your own function that processes TILT interrupt (readTiltStatus)
void accel_interrupt_callback( MMA7660::INTERRUPT p_interrupt){
	Serial.print( micros() );
	Serial.print("|accel_interrupt_callback - ");
	switch(p_interrupt)
	{
		case MMA7660::ORIENTATION_FRONT:
			Serial.print("Orient-Front");  break;
		case MMA7660::ORIENTATION_BACK:
			Serial.print("Orient-Back");   break;
		case MMA7660::ORIENTATION_UP:
			Serial.print("Orient-Up");     break;
		case MMA7660::ORIENTATION_DOWN:
			Serial.print("Orient-Down");   break;
		case MMA7660::ORIENTATION_LEFT:
			Serial.print("Orient-Left");   break;
		case MMA7660::ORIENTATION_RIGHT:
			Serial.print("Orient-Right");  break;
		case MMA7660::PULSE:
			Serial.print("Tap");           break;
		case MMA7660::SHAKE:
			Serial.print("Shake");         break;
		default:
			Serial.print("Strange - Unexpected value for interrupt. Ignoring."); break;
	}
	Serial.println("");
}


void setup() {
  Serial.begin(115200);

  pinMode(accelInterruptPin, INPUT);
  PCintPort::attachInterrupt(accelInterruptPin, isr_accel, RISING); 
  
  Serial.println("MMA7660 Testing");
  
  if( !g_Accel.init() ){
      Serial.println("PROBLEM! Initialisation failure. Module not present? Aborting...");
      delay(100);
      abort();
  }
  Serial.println("Using default sample rate (128 sps)");
  g_Accel.setFilteredCallbackFunction( accel_interrupt_callback );
  PCintPort::attachInterrupt(accelInterruptPin, isr_accel, RISING);
  
  g_Accel.enableFrontBackChangeInterrupt( );
  g_Accel.enableUpDownLeftRightChangeInterrupt( );
  g_Accel.enableShakeInterrupt( MMA7660::XY ); //Shakes in either X or Y dimensions
  g_Accel.enableTapInterrupt( MMA7660::Z );    // A tap in Z dimension

  Serial.println("Setup Complete");
  
  Serial.println("Wait for periodic readings, rotate, shake and tap");
  Serial.println("Sample rate will also be changed periodically (when sampling rate is not 128 sps then taps will not be detected)");

}

//This changes through a few different sampling rates to facilitate testing.
//You would probably want nothing like this in final code.....
unsigned int array_posn = 0;
const int sample_rates_array[] = {8,120}; //Library only supports these for now. Will abort on others.

unsigned long       accel_change_rate_last_time_ms = 0;
const unsigned long accel_change_rate_interval_ms = 9 * 1000;           // interval at which to change the internal sample rate of the accelerometer

void periodic_change_sample_rate( unsigned long current_time_ms )
{
	if((current_time_ms - accel_change_rate_last_time_ms) > accel_change_rate_interval_ms)
	{
		// save the time of this action
		accel_change_rate_last_time_ms = current_time_ms;

		Serial.print("Changing sampling rate to ");
		Serial.println( sample_rates_array[array_posn] );
		if( sample_rates_array[array_posn] == 120)
                {
                  Serial.println( "Taps will work (if enabled)" );
                }
                else
                {
                  Serial.println( "Taps will NOT work at this sampling rate" );
                }

                g_Accel.setSampleRate( sample_rates_array[array_posn] );
                array_posn = (array_posn+1) % (sizeof(sample_rates_array)/sizeof(sample_rates_array[0]));
	}
}

//Sample the axes and print the values
unsigned long       accel_sample_last_time_ms = 0;
const unsigned long accel_sample_interval_ms  = 5 * 1000;           // interval at which to sample (milliseconds) the accelerometer

void periodic_print_accel_info( unsigned long current_time_ms )
{
	if((current_time_ms - accel_sample_last_time_ms) > accel_sample_interval_ms)
	{
		// save the time of this action
		accel_sample_last_time_ms = current_time_ms;

		int l_iX = g_Accel.readAxis( MMA7660::X );
		int l_iY = g_Accel.readAxis( MMA7660::Y );
		int l_iZ = g_Accel.readAxis( MMA7660::Z );

		Serial.print("X=");
		Serial.print( l_iX );
		Serial.print(", Y=");
		Serial.print( l_iY );
		Serial.print(", Z=");
		Serial.print( l_iZ );
		Serial.println();
	}
}



void loop() {
	unsigned long current_time_ms = millis();

        periodic_change_sample_rate(current_time_ms); //Change sampling rate for testing the device at different SPS.

	periodic_print_accel_info(current_time_ms); //Read axes and print out periodically.


        //Process/filter the interrupts and raise callbacks to accel_interrupt_callback indicating a type of MMA7660::INTERRUPT as appropriate
	boolean l_zRet = g_Accel.filterProcessTime(current_time_ms); //Process (in the library) time based filtering of the interrupts.
	//Respond to accelerometer h/w interrupt
	//or a request for a fake interrupt
	if( (isr_accel_flag != 0) || l_zRet )
	{
              //Process (in the library) an actual interrupt
	      g_Accel.filterProcessInterrupt( isr_accel_flag != 0, current_time_ms );
	      isr_accel_flag=0;
	}
}

