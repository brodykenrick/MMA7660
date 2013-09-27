/**
Library for accessing Freescale MMA7660 3-axis Orientation/Motion Detection Sensor on Atmel/Arduino platform over I2C.
From https://github.com/brodykenrick/MMA7660.
Written by Brody Kenrick July 2013
Released into the public domain.
Original from https://github.com/etrombly/MMA7660 extended significantly.

Uses I2C from NinjaBlocks @ Github: https://github.com/ninjablocks/arduino/tree/master/I2C

General functionality is provided for reading the axis, modifying sampling rates and setting up the interrupts.

Extra functionality is added for filtering interrupt events and making callbacks if certain conditions are met (i.e. debouncing).

From AN3838: External Debounce Filter
"When using the MMA7660FC and sample rates higher than 32 SPS an external debounce filter
must be added to avoid flickering between positions due to involuntary human movement.
The external debounce filter must be added to the application microcontroller."

Chip details are at:
http://www.freescale.com/files/sensors/doc/data_sheet/MMA7660FC.pdf
http://www.freescale.com/files/sensors/doc/app_note/AN3837.pdf

TODO:
Move the exposed fake internal types as uint_8 to an internal subclass to have proper typedefs.
*/

#include <Arduino.h>

#include "MMA7660.h"
#include "MMA7660FC_RDHF.h" //Freescale's register definitions (a little ugly but might help in future porting efforts...)
#include <assert.h>


#define MMA7660_DEBUG //<! Define this for a verbose library
//TODO: Update using PROGMEM and F()
#ifdef MMA7660_DEBUG
  	#define MMA7660_DEBUG_PRINT(x)  	(Serial.print(x))
  	#define MMA7660_DEBUG_PRINT2(x,y)  	(Serial.print(x, y))
	#define MMA7660_DEBUG_PRINTLN(x)	(Serial.println(x))
	#define MMA7660_DEBUG_PRINTLN2(x,y)	(Serial.println(x,y))
#else
	#define MMA7660_DEBUG_PRINT(x)
	#define MMA7660_DEBUG_PRINT2(x,y)
	#define MMA7660_DEBUG_PRINTLN(x)
	#define MMA7660_DEBUG_PRINTLN2(x,y)
#endif

#if defined(MMA7660_USE_SOFT_TWI)
#define Wire (SoftWire) //Hijack Wire as SoftWire -- allows us to reuse some code as-is
//The SoftTWI library uses the read and write (8-bit) addresses -- so we need conversion functions
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#endif


//IPP - Interrupt is push-pull (vs. open drain)
//IAH - Interrupt is active high (vs. active low)
//#define ACTIVE_MODE_DEFAULT (MODE_MODE_MASK | MODE_IPP_MASK | MODE_IAH_MASK) //!<The ACTIVE MODE that is used. Code is only tested for this mode. So it is not working with auto-sleep/auto-wake (yet).
#define ACTIVE_MODE_DEFAULT (MODE_MODE_MASK | MODE_IPP_MASK ) //!<The ACTIVE MODE that is used. Code is only tested for this mode. So it is not working with auto-sleep/auto-wake (yet).



//Internal
#define PDET_PDTH_MASK (0x1F)

#define MMA7660_MODE_REGISTER_FROM_HARD_RESET (193) //!<This is what the MMA7660 MODE register is at startup (so we can check the device is present and readable)

//TODO: Add gets later...
void MMA7660::setInterruptActiveHigh(){ active_mode |=  MODE_IAH_MASK; };
void MMA7660::setInterruptActiveLow() { active_mode &= ~MODE_IAH_MASK; };
void MMA7660::setInterruptPushPull()  { active_mode |=  MODE_IPP_MASK; };
void MMA7660::setInterruptOpenDrain() { active_mode &= ~MODE_IPP_MASK; };



MMA7660::MMA7660()
{
#if defined(MMA7660_USE_NB_I2C)
	I2c.begin();
#elif defined(MMA7660_USE_ARDUINO_WIRE)
    Wire.begin();
#elif defined(MMA7660_USE_SOFT_TWI)
    Wire.attach(MMA7660_SOFT_TWI_SDA_PIN, MMA7660_SOFT_TWI_SCL_PIN);
    Wire.begin();
#endif
	active_mode = ACTIVE_MODE_DEFAULT;

    frontBackChangeInterruptEnabled 		= false;
	upDownLeftRightChangeInterruptEnabled 	= false;
	tapInterruptEnabled 					= false;

	stored_tilt_status = 0;
	samples_per_second = 120;

    process_interrupt_last_time_ms = 0;

    shake_initial_time_ms = 0;
    shake_count = 0;
    shake_sample_period_ms = 300;
    shake_count_filter = 6;

    filtered_shake_time_ms = 0;
    filtered_shake_period_ms = 1000; //One shake per second is all that is allowed.

    fn_filtered_callback = NULL;
}

//! Check that the device has a mode register that can be set/read as expected at startup.
boolean MMA7660::modeRegisterAtStartupAsExpected(void) {
    //The I2C library hides(private) the start and readAddress functions so that means a quick "is device there" function is not possible
    //So we reset the chip MODE to what it should be at hard reset and check that we read out the same thing (which only confirms that a device that can write and be read is there at the address -- but that is good enough for me...)
    //set mode to standby to change any settings
    write(MODE, 0x00);
    write(MODE, MMA7660_MODE_REGISTER_FROM_HARD_RESET);
    uint8_t startup_mode = read(MODE);
#if 0
    MMA7660_DEBUG_PRINT( "MODE: " );
    MMA7660_DEBUG_PRINT2( startup_mode, BIN );
    MMA7660_DEBUG_PRINT( " | EXPECTED: " );
    MMA7660_DEBUG_PRINTLN2( MMA7660_MODE_REGISTER_FROM_HARD_RESET, BIN );
#endif
    return (MMA7660_MODE_REGISTER_FROM_HARD_RESET == startup_mode);
}


//! Intitalise the module (after checking if it is present). Return true if all as expected.
boolean MMA7660::init() {
    boolean init_ok = modeRegisterAtStartupAsExpected();
    if( init_ok ){
	    //set mode to standby to change any settings
	    write(MODE, 0x00);
	    write(INTSU, 0x00);
	    
	    setSampleRateInternal( 120, true );
	    write(MODE, active_mode );

	    stored_tilt_status = readTiltStatus( );
	    

    }
    return init_ok;
}

#ifdef MMA7660_ENABLE_INCOMPLETE_CODE
void MMA7660::setPowerMode(int mode) {
	assert(false);
}
#endif /*MMA7660_ENABLE_INCOMPLETE_CODE*/


//!Set the sampling rate (in Active mode -- we aren't using auto-wake yet).....
void MMA7660::setSampleRate(int samplesPerSecond) {
    setSampleRateInternal(samplesPerSecond, false);
}

//!Set the sampling rate (in Active mode -- we aren't using auto-wake yet).....
void MMA7660::setSampleRateInternal(int samplesPerSecond, boolean stateAlreadyNotActive) {
	if(!stateAlreadyNotActive)
	{
		write(MODE, 0x00);
	}

	if(samplesPerSecond == 120)
	{
		samples_per_second = samplesPerSecond;

		SR_STR sr;
		sr.Byte = 0; //120 active
		sr.Merged.grpFILT = 0b111; //8 (total) samples need to be the same before an interrupt on F/B | U/D/L/R

		write(SR, sr.Byte);
	}
	else
	if(samplesPerSecond == 8)
	{
		samples_per_second = samplesPerSecond;

		SR_STR sr;
		sr.Byte = 0;

		sr.Merged.grpAMSR = 0b100; //8 sps
		sr.Merged.grpAWSR = 0b10;  //8 sps
		sr.Merged.grpFILT = 0b01; //2 (total) samples need to be the same before an interrupt on F/B | U/D/L/R

		write(SR, ( sr.Byte ));
	}
	else
	{
		assert(false);
	}
	if(!stateAlreadyNotActive)
	{
		write(MODE, active_mode );
	}
}

//! Retrieve the devices set samples per second.
int MMA7660::getSampleRate( void )
{
	return samples_per_second;
}

//! Update the Interrupt Set-up register
void MMA7660::rewriteIntSURegister(uint8_t valueToBitwiseOR, boolean stateAlreadyNotActive) {
	if(!stateAlreadyNotActive)
	{
		write(MODE, 0x00);
	}
  	write(INTSU, readInterruptStatus() | valueToBitwiseOR ); //turn on by ORing
  	if(!stateAlreadyNotActive)
	{
  		write(MODE, active_mode );
	}
}

//8,25 worked but was too sensitive
#define TAP_THRESHOLD 	(16)
#define TAP_DEBOUNCE 	(32)

//! Tap interrupt enable.
//! Filter settings are hardcoded.
//! Only works in active mode (i.e. 120 samps/sec) - so this changes the sample rate
//!NOTE: Taps CANNOT trigger a device wake-up.
void MMA7660::enableTapInterrupt( AXIS pAxis ) {
	uint8_t axis = 0;
	switch(pAxis)
	{
		case MMA7660::X: 	axis = PDET_XDA_MASK; break;
		case MMA7660::Y: 	axis = PDET_YDA_MASK; break;
		case MMA7660::Z: 	axis = PDET_ZDA_MASK; break;
		default: 	assert(false); break;
	}
	write(MODE, 0x00);
	rewriteIntSURegister(INTSU_PDINT_MASK); //turn on tap/pulse interrupt
	//Requires Samples ==120 -->  AMSR [2:0] = 000 in the SR (0x08) register
	setSampleRateInternal( 120, true );
	//Enable axis tap and set threshold to N taps (max of 31)
	write(PDET, axis | (PDET_PDTH_MASK & TAP_THRESHOLD ) );
	//Set Tap detection debounce filter to N measurements (max of 255)
	write(PD, (PD_PD_MASK & TAP_DEBOUNCE ));
	tapInterruptEnabled = true;
	write(MODE, active_mode );
}

//! Shake interrupt enable.
void MMA7660::enableShakeInterrupt( AXIS pAxis ) {
	write(MODE, 0x00);
	uint8_t axis = 0;
	switch(pAxis)
	{
		case MMA7660::X: 	axis = INTSU_SHINTX_MASK; break;
		case MMA7660::Y: 	axis = INTSU_SHINTY_MASK; break;
		case MMA7660::Z: 	axis = INTSU_SHINTZ_MASK; break;
		case MMA7660::XY: 	axis = (INTSU_SHINTX_MASK | INTSU_SHINTY_MASK); break;
		case MMA7660::XZ: 	axis = (INTSU_SHINTX_MASK | INTSU_SHINTZ_MASK); break;
		case MMA7660::YZ: 	axis = (INTSU_SHINTY_MASK | INTSU_SHINTZ_MASK); break;
		case MMA7660::XYZ: 	axis = (INTSU_SHINTX_MASK | INTSU_SHINTY_MASK | INTSU_SHINTZ_MASK); break;
		default: 	assert(false); break;
	}
	rewriteIntSURegister(axis);
	write(MODE, active_mode );
}

//!<Front:Back orientation change interrupt enable
void MMA7660::enableFrontBackChangeInterrupt( ) {
	write(MODE, 0x00); //No sleep count
	rewriteIntSURegister(INTSU_FBINT_MASK); //Front:back interrupts enabled
    frontBackChangeInterruptEnabled = true;
	write(MODE, active_mode );
}

//!<Up:Down:Left:Right orientation change interrupt enable
void MMA7660::enableUpDownLeftRightChangeInterrupt( ) {
	write(MODE, 0x00); //No sleep count
	rewriteIntSURegister(INTSU_PLINT_MASK); //Up:Down:Left:Right interrupts enabled
	upDownLeftRightChangeInterruptEnabled = true;
	write(MODE, active_mode );
}

#if MMA7660_ENABLE_INCOMPLETE_CODE
//!<Wake up causes an interrupt. We don't support sleep modes so this is not much use presently.
void MMA7660::enableExitAutoSleepInterrupt( ) {
	write(MODE, 0x00); //No sleep count
	rewriteIntSURegister(INTSU_ASINT_MASK);
	write(MODE, active_mode );
}
#endif /*MMA7660_ENABLE_INCOMPLETE_CODE*/


#if MMA7660_ENABLE_INCOMPLETE_CODE
//! Automatic (after each reading) interrupt enable.
//Incomplete as interrupt filtering and callbacks aren't done
void MMA7660::enableAutomaticInterrupt( ) {
	write(MODE, 0x00);
	rewriteIntSURegister(INTSU_GINT_MASK);
	write(MODE, active_mode );
}
#endif /*MMA7660_ENABLE_INCOMPLETE_CODE*/

//!<Disable all interrupts
void MMA7660::disableAllInterrupts() {
	write(MODE, 0x00);
	write(INTSU, 0x00);
	tapInterruptEnabled = false;
    frontBackChangeInterruptEnabled = false;
	upDownLeftRightChangeInterruptEnabled = false;
	write(MODE, active_mode );
}


//! Read the tilt status register as a byte (if you are doing your own filtering/debugging). This also clears an interrupt at the device.
uint8_t MMA7660::readTiltStatus() {
    TILT_STR l_tilt;

	//check if the alert bit is set, if so re-read
	do
	{
		l_tilt.Byte = read(TILT);
	} while(l_tilt.Bits.ALERT);

	return l_tilt.Byte;
}


//TODO: Make private later
//TODO: Hide the details of the values here
uint8_t MMA7660::readInterruptStatus() {
	return read(INTSU);
}

//! Reads the single axis and returns a signed integer
int MMA7660::readAxis( AXIS pAxis ) {
	XOUT_STR out; //Each of X, Y, Z have the same structure
	uint8_t axis = 0;

	switch(pAxis)
	{
		case MMA7660::X:  axis = XOUT; break;
		case MMA7660::Y:  axis = YOUT; break;
		case MMA7660::Z:  axis = ZOUT; break;
		default: assert(false); return 0; //This means trying to read multiple axes or invalid....
	}

	//check if the alert bit is set, if so re-read
	do
	{
		out.Byte = read(axis);
	}while( out.Bits.ALERT );

	uint8_t data = out.Merged.grpXOUT;
	//if it's a negative value convert the two's complement
	if(data & 0x20)
	{
		data += 0xC0; //the register returns a 6 bit number, fix for uint8_t
		data = ~data;
		data++;
		return (int) (data * -1);
	}
	return (int)data;
}


//! Set the filtered call back function to be called when an interrupt is generated and the filterProcessTime/Interrupt functions are used to process it.
void MMA7660::setFilteredCallbackFunction( void (*fn_filtered_callback)(MMA7660::INTERRUPT) )
{
	this->fn_filtered_callback = fn_filtered_callback;
}

//! Still doing some stuff (so you shouldn't for example sleep just yet)
boolean MMA7660::filterProcessingStillUnderway( unsigned long current_time_ms )
{
    return (shake_count != 0);
}

//! Allows for some checks to be done for our debounce filtering.
//! Returns true if we want to execute a "fake" interrupt to allow for debouncing of tilt status.
boolean MMA7660::filterProcessTime( unsigned long current_time_ms )
{
	//Ignore all shakes until we end this period
	if( ((current_time_ms - filtered_shake_time_ms) < filtered_shake_period_ms ) )
	{
		shake_count = 0;
	}

	boolean l_zRet = false;
	//This is a timeout for checking if enough shakes happened in a period to consider it to really have been shaking
	if( (shake_count > 0) && ((current_time_ms - shake_initial_time_ms) > shake_sample_period_ms ) )
	{
		MMA7660_DEBUG_PRINT("Shake period up : ");
		MMA7660_DEBUG_PRINT(shake_count);
		MMA7660_DEBUG_PRINT("/");
		MMA7660_DEBUG_PRINT( shake_count_filter );
		MMA7660_DEBUG_PRINT(". -> ");
		if(shake_count >= shake_count_filter)
		{
			shake_count = 0;
			assert( fn_filtered_callback );
			fn_filtered_callback( MMA7660::SHAKE );
			filtered_shake_time_ms = current_time_ms;
		}
		else
		{
			MMA7660_DEBUG_PRINTLN(" Cancelled....");
			shake_count = 0;
		}
	}

	return l_zRet;
}


//! Process the hardware interrupt and filter as appropriate - generating callbacks when conditions are met.
void MMA7660::filterProcessInterrupt(boolean realInterrupt,
		unsigned long current_time_ms)
{
	TILT_STR tilt;
	process_interrupt_last_time_ms = current_time_ms;

	//Reading of the TILT register clears the MMA7660 to send another interrupt.
	tilt.Byte = readTiltStatus();
	TILT_STR * stored_tilt = (TILT_STR *) &stored_tilt_status;

	if(realInterrupt)
	{
		//MMA7660_DEBUG_PRINT("Real int: ");
	}
	
	if(frontBackChangeInterruptEnabled)
	{
        //Front or back
	    if (tilt.Merged.grpBAFRO != stored_tilt->Merged.grpBAFRO) {
		    //MMA7660_DEBUG_PRINTLN("F/B int");
		    MMA7660::INTERRUPT type_of_int = INTERRUPT_INVALID;
		    if(tilt.Merged.grpBAFRO == 0b01)
		    {
    		    type_of_int=ORIENTATION_FRONT;
		    }
		    else if(tilt.Merged.grpBAFRO == 0b10)
		    {
    		    type_of_int=ORIENTATION_BACK;
		    }
		    if(type_of_int != INTERRUPT_INVALID)
		    {
                fn_filtered_callback( type_of_int );
            }
	    }
    }
	
    if(upDownLeftRightChangeInterruptEnabled)
    {
        //Up, Down, Left or Right
	    if (tilt.Merged.grpPOLA != stored_tilt->Merged.grpPOLA) {
		    //MMA7660_DEBUG_PRINTLN("U/D/L/R int");
		    MMA7660::INTERRUPT type_of_int = INTERRUPT_INVALID;
		    switch(tilt.Merged.grpPOLA)
		    {
                case 0b001: type_of_int=ORIENTATION_LEFT;   break;
                case 0b010: type_of_int=ORIENTATION_RIGHT;  break;
                case 0b101: type_of_int=ORIENTATION_DOWN;   break;
                case 0b110: type_of_int=ORIENTATION_UP;     break;
                default: break;
		    }
		    if(type_of_int != INTERRUPT_INVALID)
		    {
                fn_filtered_callback( type_of_int );
            }
	    }
	}

    if(tapInterruptEnabled) //TODO: Check the data sheet better. This should not be seen unless enabled -- BUT they are....
    {
	    if( tilt.Bits.PULSE )
	    {
		    //TODO: Filter? Or just trust the internal accel filters....
		    MMA7660_DEBUG_PRINTLN("Tap (no ext. filter)...");
		    assert( fn_filtered_callback );
		    fn_filtered_callback( MMA7660::PULSE );
	    }
	}


	//Shake should be if more than a certain number of shakes are detected in a period of time
	if( tilt.Bits.SHAKE )
	{
		//MMA7660_DEBUG_PRINT("Shake int! ");
		if(shake_count > 0 )
		{
			shake_count++;
		}
		else
		{
			if( samples_per_second <= 8 )
			{
				//If sampling rate is low though it should be trusted...
				MMA7660_DEBUG_PRINT("Shake (low SPS)...");
				assert( fn_filtered_callback );
				fn_filtered_callback( MMA7660::SHAKE );
			}
			else
			//We setup a time based counter....
			if( samples_per_second <= 64 )
			{
				shake_count_filter = 8;
				//Starting the time period and the counter
				shake_count = 1;
				shake_initial_time_ms = current_time_ms;
				shake_sample_period_ms = 300;
			}
			else
			if( samples_per_second <= 128 )
			{

				shake_count_filter = 15;
				//Starting the time period and the counter
				shake_count = 1;
				shake_initial_time_ms = current_time_ms;
				shake_sample_period_ms = 300;
			}
		}
		//MMA7660_DEBUG_PRINT(" : ");
		//MMA7660_DEBUG_PRINTLN( shake_count );
	}

	//Update our tilt register to allow us to determine what change in orientation has occurred.
	stored_tilt_status = tilt.Byte;
}

//! Write to the device
void MMA7660::write(const uint8_t registerAddress, const uint8_t data)
{
#if defined(MMA7660_USE_NB_I2C)
    I2c.write((uint8_t)MMA7660_CTRL_ID_DEFAULT, registerAddress, data);
#elif defined(MMA7660_USE_ARDUINO_WIRE) || defined(MMA7660_USE_SOFT_TWI)
//    MMA7660_DEBUG_PRINT( "write(0x" );
//    MMA7660_DEBUG_PRINT2( registerAddress, HEX );
//    MMA7660_DEBUG_PRINT( ",0x" );
//    MMA7660_DEBUG_PRINT2( data, HEX );
//    MMA7660_DEBUG_PRINTLN( ")" );
#if defined(MMA7660_USE_ARDUINO_WIRE)
    Wire.beginTransmission(MMA7660_CTRL_ID_DEFAULT);
#elif defined(MMA7660_USE_SOFT_TWI)
    Wire.beginTransmission( SLA_W(MMA7660_CTRL_ID_DEFAULT) );
#endif
    Wire.write(registerAddress);
    Wire.write(data);
    Wire.endTransmission();

#endif

}

//! Read a byte from the device
uint8_t MMA7660::read(const uint8_t registerAddress)
{
	//TODO: Add in alert re-read here. Selectively for those reads with an alert field (axes and tilt status)
#if defined(MMA7660_USE_NB_I2C)
    I2c.read((uint8_t)MMA7660_CTRL_ID_DEFAULT, registerAddress,(uint8_t) 0x01);
	return I2c.receive();
#elif defined(MMA7660_USE_ARDUINO_WIRE) || defined(MMA7660_USE_SOFT_TWI)
    //Arduino WIRE does not support "repeated starts"
    //SO we just read all data and select the byte/register we want
    //http://forum.arduino.cc/index.php?topic=80845.0
#define NUM_REGISTERS (PD+1)

#if defined(MMA7660_USE_ARDUINO_WIRE)
    Wire.beginTransmission(MMA7660_CTRL_ID_DEFAULT);
#elif defined(MMA7660_USE_SOFT_TWI)
    Wire.beginTransmission( SLA_W(MMA7660_CTRL_ID_DEFAULT) );
#endif
    Wire.write( 0x00 );  // We read from the start of the register space
    Wire.endTransmission();
#if defined(MMA7660_USE_ARDUINO_WIRE)
    Wire.requestFrom(MMA7660_CTRL_ID_DEFAULT, NUM_REGISTERS); // and read all registers
#elif defined(MMA7660_USE_SOFT_TWI)
    Wire.requestFrom( SLA_R(MMA7660_CTRL_ID_DEFAULT), NUM_REGISTERS); // and read all registers
#endif
    
    uint8_t read_byte = 0xFF;
    for(int j=0; j<NUM_REGISTERS; j++)
    {
        uint8_t read_byte_tmp = Wire.read();
        if(j == registerAddress)
        {
            read_byte = read_byte_tmp;
        }
    }

    return read_byte;
#endif

}

