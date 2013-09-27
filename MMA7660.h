/*
Library for accessing Freescale MMA7660 3-axis Orientation/Motion Detection Sensor on Atmel/Arduino platform over I2C.
From https://github.com/brodykenrick/MMA7660.
Written by Brody Kenrick July 2013
Released into the public domain.

See .cpp file for more information.

Original from https://github.com/etrombly/MMA7660 extended significantly.

Function comments are with the implementation in the .cpp and not in this header.
*/
#ifndef MMA7660_h
#define MMA7660_h

#include <Arduino.h>

//Select which TWI/I2C/Wire interface should be used. Only one of these should be selected.
#define MMA7660_USE_NB_I2C        //!< I2C from NinjaBlocks @ Github: https://github.com/ninjablocks/arduino/tree/master/I2C
//#define MMA7660_USE_ARDUINO_WIRE
//#define MMA7660_USE_SOFT_TWI


#if defined(MMA7660_USE_NB_I2C)

#include <I2C.h>
#undef MMA7660_USE_ARDUINO_WIRE
#undef MMA7660_USE_SOFT_TWI

#elif defined(MMA7660_USE_ARDUINO_WIRE)

#include <Wire.h>
#undef MMA7660_USE_NB_I2C
#undef MMA7660_USE_SOFT_TWI

#elif defined(MMA7660_USE_SOFT_TWI)

#include <SoftTWI.h>
#undef MMA7660_USE_NB_I2C
#undef MMA7660_USE_ARDUINO_WIRE

// Set soft TWI pin numbers for your configuration.
#define  MMA7660_SOFT_TWI_SDA_PIN ((uint8_t)SDA)
#define  MMA7660_SOFT_TWI_SCL_PIN ((uint8_t)SCL)

#else

#error "Wire/I2C/TWI not selected!"

#endif


#define MMA7660_CTRL_ID_DEFAULT (0x4C) //!<This is the factory pre-configured I2C address for an MMA7660

//#define MMA7660_ENABLE_INCOMPLETE_CODE //!<Turn on some experimental code....


/// Library for accessing the Freescale MMA7660 3-axis Orientation/Motion Detection Sensor
class MMA7660
{

  // user-accessible "public" interface
	public:
		enum AXIS { X, Y, Z, XY, XZ, YZ, XYZ, AXIS_INVALID }; //!< For reading the value or setting an interrupt from/for one or more axes
		enum INTERRUPT {PULSE, //!< Pulse/Tap in one of the dimensions enabled (indistinguisahble).
		                SHAKE, //!< Shake in one of the dimensions enabled (indistinguisahble)
		                ORIENTATION_FRONT, ORIENTATION_BACK,
		                ORIENTATION_UP, ORIENTATION_DOWN, ORIENTATION_LEFT, ORIENTATION_RIGHT,
		                AUTO, INTERRUPT_INVALID }; //!<Types of interrupts from our callback function
		//TODO: A "getOrientation" function

		MMA7660();
		
		void setInterruptActiveHigh(); //!< vs. active LOW (default)
		void setInterruptActiveLow();
		void setInterruptPushPull();   //!< vs. open drain (default)
		void setInterruptOpenDrain();
		
		boolean init();

		void setSampleRate(int samplesPerSecond);
		int  getSampleRate();

		//Interrupts are hardware and the library does NOT intercept them.
		//It is expected that the sketch gets them and hands them back to the library for filtering if it is desired.
		void enableTapInterrupt( AXIS );
		void enableShakeInterrupt( AXIS );
		void enableFrontBackChangeInterrupt( );
		void enableUpDownLeftRightChangeInterrupt( );
#if MMA7660_ENABLE_INCOMPLETE_CODE
		void enableExitAutoSleepInterrupt( );
		void enableAutomaticInterrupt( );
#endif /*MMA7660_ENABLE_INCOMPLETE_CODE*/
        void disableAllInterrupts();



		int     readAxis( AXIS );
		
#if MMA7660_ENABLE_INCOMPLETE_CODE
		void setPowerMode(int mode);
#endif /*MMA7660_ENABLE_INCOMPLETE_CODE*/

		//Todo: Make these for friends. Only want subclasses that don't do filtering...
		//TODO: then should also make our filtering a subclass by that logic.....
		uint8_t readTiltStatus();
		uint8_t readInterruptStatus();

	private:
        boolean modeRegisterAtStartupAsExpected(void);
    	uint8_t read(const uint8_t registerAddress);
		void 	write(const uint8_t registerAddress, const uint8_t data);
        void 	rewriteIntSURegister(uint8_t valueToBitwiseOR, boolean stateAlreadyNotActive = true);
        
        void    setSampleRateInternal(int samplesPerSecond, boolean stateAlreadyNotActive = false);

        int 			samples_per_second;
        
        uint8_t active_mode;

        boolean tapInterruptEnabled;        //Need to see if these are enabled - as the device seems to raise taps even when disabled......
        
        boolean frontBackChangeInterruptEnabled;        //Need to see if these are enabled - as we can't tell the difference after we check the TILT reg
		boolean upDownLeftRightChangeInterruptEnabled;

        //Filtering on interrupts/callbacks after here
	public:
        //For filtering the h/w interrupts and generating callbacks
        //Process the hardware ints inside the library (with a timebase provided)
		//Don't cll these or have a callback here if you want to process the interrupts all by yourself
        void 	setFilteredCallbackFunction( void (*fn_filtered_callback)(MMA7660::INTERRUPT) );
        boolean filterProcessTime( unsigned long current_time_ms );
        void    filterProcessInterrupt( boolean realInterrupt, unsigned long current_time_ms );
        
        boolean filterProcessingStillUnderway( unsigned long current_time_ms );

	private:

        void (*fn_filtered_callback)(MMA7660::INTERRUPT); //!< This is the callback function we'll call with an indication of the interrupt generated after we proces in the library.

        uint8_t 		stored_tilt_status; //!< Note matches the internal struct/type for TILT_STR (to hide implementation details).

        unsigned long   process_interrupt_last_time_ms;



        unsigned long       shake_initial_time_ms;
        unsigned long       shake_sample_period_ms; //Could change depending on sampling rate....
        int                 shake_count; //This will need to scale depending on sample rate
        int                 shake_count_filter; //could change depending on sampling rate....

        unsigned long       filtered_shake_period_ms; //Time between filtered shakes
        unsigned long       filtered_shake_time_ms;

};

#endif
