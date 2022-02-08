/*
  This code is for a drone-mounted chlorophyll fluorescence sensor.
  
  The circuit: Arduino Nano; L293D dual half h-bridge for switching laser and shutter using pins 4-9; 
  differential input channel on A0/A1 for the fluorescence detector; differential input channel on A2/A3
  for the reference detector; communication with Raspberry Pi via ROS on UART; SD card logger on SPI (pins 10-13).

  Pin 8 and 9 are controlled by tasks and events through the PPI. Pin 9 turns the laser on and off;
  pin 8 is just an indicator.

  The Nano reads the signal from the fluorescence detector and the reference detector. 
  Each measurement is calculated by subtracting the signal level with the laser off. 
  The resulting values are published to ROS topics which are monitored by the Pi.
  All SAADC data is stored to the SD card (5 kB/s).

  ADC Settings:
        - 12 bit resolution (Oversample only possible with single channel)
        - input range 0-3.3 V (VDD)
        - channel 0: differential input mode, positive input A0, negative input A1
        - channel 1: differential input mode, positive input A2, negative input A3
  
  Sampling frequency: 10 Hz

  The laser is switched at 20 Hz (50ms on/50ms off). During each 50 ms on/off period,
  128 samples will be taken on each channel and stored in a data array (total sampling 
  rate = 256S/0.05s = 5,120 S/s). All 128 samples will be summed into one 32-bit value

  There are 9 dignostic tests which must be passed in order to proceed with take-off:
    1. Temperature inside sensor housing is within limits
    2. Humidity inside sensor housing is within limits
    3. Dark noise for reference detector is within limits
    4. Dark noise for fluorescence detector is within limits
    5. Laser intensity is within limits
    6. Background fluorescence is within limits
    7. No ambient light inside sensor housing when laser is off
    8. Ambient light inside sensor housing when laser is on is within limits
    9. Fluorescence from the calibration reference standard is within limits
  A calibration value wil also be calculated based on the fluorescence intensity of the calibration reference standard.  
  
*/

#include "startup_definitions.h"
#include <Arduino.h>
#include <ros.h>                  // ROS main library and data type libraries
#include <std_msgs/Int16.h>       
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "mbed.h"                 // library for NRF52480 (uc chip)
#include <SPI.h>                  
#include "SdFat.h"                // SD card library
#include "RTClib.h"               // RTC library (real time clock for file timestamps)
#include <Arduino_HTS221.h>       // library for temp and humidity sensor
#include <Arduino_APDS9960.h>     // library for brightness sensor

ros::NodeHandle nh;               // create node handle object for ROS

SdExFat sd;                       // create SD and File objects with exFAT file system
ExFile file;

RTC_DS1307 rtc;                   // create Real Time Clock object


#define ADC_BUFFER_SIZE   256     // the number of 16 bit samples to store in the data buffer
#define NUM_CHANNELS 2            // 2 ADC channels (fluor and ref)

#define reportFreq  10            // sets frequency of updates to ROS in Hz

#define laserPeriod 50            // laser on (or off) time = half of the laser period in ms
#define onDelay 3000              // time it takes for laser to fully turn on in us
#define offDelay 2500             // time it takes for laser to fully turn off in us

const int numSamples = ADC_BUFFER_SIZE/NUM_CHANNELS;       // number of samples per sampling period (laser on or off)
const int sampleTime = (1000*(laserPeriod-3))/numSamples;  // amount of time per sample in us (laser period padded with 3 ms delay)

volatile nrf_saadc_value_t adcBuffer[ADC_BUFFER_SIZE] = {0};   // result buffer, initiallized to 0
volatile int16_t * bufferPtr = adcBuffer;                      // pointer to the result buffer
const void* filePtr = (const void*) &adcBuffer;                // pointer for writing to SD card

volatile int adcFlag = 0;          // flag set by SAADC ISR when buffer is full
volatile bool writeFlag = false;        // flag for "ready to write to SD"

volatile bool errorFlag = false;   // flag for tracking errors
 
volatile int16_t sensorMode = 0;   // 0 = idle, 1 = recording, 2 = single calibration, 3 = continuous calibration
bool active = false;               // true while sensor is running

int32_t fluorOn, fluorOff, refOn, refOff; // variables for recording data
int32_t fluor, ref;                       // variables for calculating results
uint32_t lastReading;                     // used to time humidity and temp readings

const int shutterEN = 4;           // pin 4 is the enable pin for the shutter
const int shutterPos = 5;
const int shutterNeg = 6; 


// Callback function for subscriber input on ROS
void messageCb( const std_msgs::Int16& mode){
  sensorMode = mode.data;                                     // Set flag to start or stop recording (interrupts the while(recording) loop
}


// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = rtc.now();

  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(now.year(), now.month(), now.day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(now.hour(), now.minute(), now.second());

  // Return low time bits in units of 10 ms, 0 <= ms10 <= 199.
  *ms10 = now.second() & 1 ? 100 : 0;
}


ros::Subscriber<std_msgs::Int16> mode("mode", messageCb );    // Declare subscriber to messages on "mode" topic, trigger callback function

std_msgs::Int32 fluorMsg;                                     // Set up data type for fluorescence data message
std_msgs::Int32 refMsg;                                       // Set up data type for reference data message
std_msgs::Float32 temperature;                                // Set up data type for temp data message
std_msgs::Float32 humidity;                                   // Set up data type for humidity data message
std_msgs::String myErrorMsg;                                  // Set up error message
std_msgs::Int32 calibrationMsg;
std_msgs::Int16 diagnostic;

ros::Publisher fluorPub("fluor", &fluorMsg);             // Declare publisher of fluorescence data message on "fluor_data" topic
ros::Publisher refPub("ref", &refMsg);                   // Declare publisher of reference data message on "ref_data" topic
ros::Publisher tempPub("temp", &temperature);                 // Declare publisher of temp data message on "temp" topic
ros::Publisher humPub("humidity", &humidity);                 // Declare publisher of humidity data message on "humidity" topic
ros::Publisher errorPub("error", &myErrorMsg);                     // Set up publisher of error topic
ros::Publisher calibrationPub("calibration", &calibrationMsg);
ros::Publisher diagnosticPub("diagnostic", &diagnostic);


void setup(){
  pinMode(shutterEN, OUTPUT);
  pinMode(shutterPos, OUTPUT);
  pinMode(shutterNeg, OUTPUT);

  HTS.begin();                   // Initialize temp and humidity sensor
  APDS.begin();                  // Initialize brightness sensor
  rtc.begin();                   // initialize RTC
  sd.begin(SS);                  // initialize SD
  
  FsDateTime::setCallback(dateTime); // Set file datetimestamp
  
  nh.initNode();                 // Initialize node handle
  nh.advertise(fluorPub);        // Start advertising/publishing on "fluor_data" topic
  nh.advertise(refPub);          // Start advertising/publishing on "ref_data" topic
  nh.advertise(tempPub);         // Start advertising/publishing on "temp" topic
  nh.advertise(humPub);          // Start advertising/publishing on "humidity" topic
  nh.advertise(errorPub);        // Advertise error topic
  nh.advertise(calibrationPub);
  nh.advertise(diagnosticPub);
  nh.subscribe(mode);            // Initialize subscriber to messages on "record" topic

  initADC();
  initLaserTimer();
  initSamplingTimer(sampleTime);
  initGPIOTE();
  initPPI();
}



void loop() {  
/*---------------------------------------------------------------------------------------------    
                                        Idle Mode
----------------------------------------------------------------------------------------------- */  
  while( sensorMode == 0 ){                     // if Pi sets the "mode" flag on ROS to 0 (Idle mode)
    if( active ){                               // if the sensor is still active
      shutdownSensor();                         // shut it down
    }
    if( millis() > (lastReading + 5000) ){      // if it has been more than 5 seconds since last reading 
      getTempHum();
      lastReading = millis();
    }
    if( errorFlag ){
      errorPub.publish( &myErrorMsg );               // publish any error messages
      errorFlag = false;
    }
    nh.spinOnce();                              // update ROS node
  } // end of Idle Mode
  
/*---------------------------------------------------------------------------------------------    
                                Record Mode
----------------------------------------------------------------------------------------------- */ 
  while( sensorMode == 1 ){                     // if Pi sets the "mode" flag on ROS to 1 (Record mode)
    if( !active ){                              // if the sensor isn't active
      startSensor();
      createNewFile();
      
      NRF_TIMER3->TASKS_START = 1;              // start Timer 3
    }

    if( errorFlag ){
      sensorMode = 0;
    }

    switch( adcFlag ){
      case 0:                                   // while the flag is set to 0, do nothing
        break;
        
      case 1:                                   // when the flag changes to 1, sum results from On data
        getData( fluorOn, refOn );
        adcFlag++;                              // increment flag (go to case 2)
        writeFlag = true;
        break;
      
      case 2:                                   // while the flag is set to 2, store data to SD card    
        break;
      
      case 3:                                   // when the flag changes to 3, sum results from Off data then send data to ROS
        getData( fluorOff, refOff );
        reportResults( fluorOn, fluorOff, refOn, refOff );
        writeFlag = true;                       // set writeFlag
        break;
      
      default:
        adcFlag = 0;
        break;
    }
    
    if( writeFlag ){
      file.write(filePtr,ADC_BUFFER_SIZE*4); // write data to SD card as 512 byte chunk
      writeFlag = false;                     // clear flag
    }
    
    if( millis() > (lastReading + 60000) ){ // if it has been more than 1 minute since last reading 
      getTempHum();
    }
    
    nh.spinOnce();                          // update ROS node
  } // end of Record mode

  file.close();                             // sync and close file to save it on the SD card


/*----------------------------------------------------------------------------------------------    
                                Calibration
------------------------------------------------------------------------------------------------ */
  while( sensorMode == 2 ){                     // if Pi sets the "mode" flag on ROS to 2 (Calibrate mode)
    uint16_t diagnosticCode = 0x0000;
    uint32_t calibration;
    startupRoutine(diagnosticCode, calibration);

    calibrationMsg.data = calibration;
    diagnostic.data = diagnosticCode;

    calibrationPub.publish( &calibrationMsg );
    diagnosticPub.publish( &diagnostic );

    nh.spinOnce();                              // update ROS node

    sensorMode = 0;                             // go to Idle mode
    
  } // end of calibration mode
  

/*----------------------------------------------------------------------------------------------    
                           Laser and Shutter Controls
------------------------------------------------------------------------------------------------ */
  if( sensorMode == 3 ){
    NRF_GPIOTE->TASKS_CLR[0] = 1;                 // turn laser off
    sensorMode = 0;
  }
  
  if( sensorMode == 4 ){
    NRF_GPIOTE->TASKS_SET[0] = 1;                 // turn laser on
    sensorMode = 0;
  }  
  if( sensorMode == 5 ){
    openShutter();
    sensorMode = 0;
  }

  if( sensorMode == 6 ){
    closeShutter();
    sensorMode = 0;
  }


/*----------------------------------------------------------------------------------------------    
                                Live Data Feed
------------------------------------------------------------------------------------------------ */
  while( sensorMode == 7 ){                     
    int32_t fluorLive, refLive;
    collectOneCycle( fluorLive, refLive );                 
          
    fluorMsg.data = fluorLive;                  // update fluorMsg ROS message
    refMsg.data = refLive;                      // update refMsg ROS message
    fluorPub.publish( &fluorMsg );              // publish fluor on the "fluor_data" ROS topic
    refPub.publish( &refMsg );                  // publish ref on the "ref_data" ROS topic

    nh.spinOnce();                              // update ROS node
  } //end live data feed
  
  if( sensorMode > 7){
    sensorMode = 0;    
  }

} // end of loop


/*----------------------------------------------------------------------------------------------    
                                Buffer full ISR
------------------------------------------------------------------------------------------------ */
extern "C" void SAADC_IRQHandler_v( void ){
  if ( NRF_SAADC->EVENTS_END != 0 ){
    NRF_SAADC->EVENTS_END = 0;
    NRF_TIMER2->TASKS_STOP = 1;            // stop Timer 2 to halt SAADC
    NRF_TIMER2->TASKS_CLEAR = 1;
    adcFlag++;                             // when SAADC result buffer is full, increment the flag
  }
}


/*----------------------------------------------------------------------------------------------    
                                Set up SAADC
------------------------------------------------------------------------------------------------ */
void initADC(){
  nrf_saadc_disable();

  nrf_saadc_channel_config_t  settings0;                  // Create settings object for channel 0 (fluorescence detector)
    settings0.resistor_p = NRF_SAADC_RESISTOR_DISABLED ;
    settings0.resistor_n = NRF_SAADC_RESISTOR_DISABLED ;
    settings0.gain       = NRF_SAADC_GAIN1_4 ;            // 1/4x Gain
    settings0.reference  = NRF_SAADC_REFERENCE_INTERNAL ; // Internal 0.6 V Reference
    settings0.acq_time   = NRF_SAADC_ACQTIME_3US ;
    settings0.mode       = NRF_SAADC_MODE_DIFFERENTIAL ;  // Differential input
    settings0.burst      = NRF_SAADC_BURST_DISABLED ;
    settings0.pin_p      = NRF_SAADC_INPUT_AIN2 ;         // Nano A0
    settings0.pin_n      = NRF_SAADC_INPUT_AIN3 ;         // Nano A1

  nrf_saadc_channel_config_t  settings1;                  // Create settings object for channel 1 (reference detector)
    settings1.resistor_p = NRF_SAADC_RESISTOR_DISABLED ;
    settings1.resistor_n = NRF_SAADC_RESISTOR_DISABLED ;
    settings1.gain       = NRF_SAADC_GAIN1_4 ;            // 1/4x Gain
    settings1.reference  = NRF_SAADC_REFERENCE_VDD4 ;     // VDD/4 Reference
    settings1.acq_time   = NRF_SAADC_ACQTIME_3US ;
    settings1.mode       = NRF_SAADC_MODE_DIFFERENTIAL ;  // Differential input
    settings1.burst      = NRF_SAADC_BURST_DISABLED ;
    settings1.pin_p      = NRF_SAADC_INPUT_AIN5 ;         // Nano A3
    settings1.pin_n      = NRF_SAADC_INPUT_AIN6 ;         // Nano A2
      
  nrf_saadc_resolution_t resolution = NRF_SAADC_RESOLUTION_12BIT ;

  nrf_saadc_oversample_t oversample = NRF_SAADC_OVERSAMPLE_DISABLED ;

  nrf_saadc_channel_init( 0 , &settings0 );               // Set up SAADC channel 0
  nrf_saadc_channel_init( 1 , &settings1 );               // Set up SAADC channel 1
  
  NRF_SAADC->RESULT.MAXCNT = ADC_BUFFER_SIZE;             // Set up Result buffer
  NRF_SAADC->RESULT.PTR = ( uint32_t ) bufferPtr;
  
  nrf_saadc_resolution_set( resolution );
  
  nrf_saadc_oversample_set( oversample );

  NRF_SAADC->EVENTS_END = 0;
  nrf_saadc_int_enable( NRF_SAADC_INT_END );              // Set up Interrupt
  NVIC_SetPriority( SAADC_IRQn, 1UL );
  NVIC_EnableIRQ( SAADC_IRQn );

  nrf_saadc_enable();
  
  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;                   // Calibrate SAADC
  while ( NRF_SAADC->EVENTS_CALIBRATEDONE == 0 );
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while ( NRF_SAADC->STATUS == ( SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos ) );
}


/*----------------------------------------------------------------------------------------------    
                                Set up Timers
------------------------------------------------------------------------------------------------ */
void stopTimers(){
  NRF_TIMER3->TASKS_STOP = 1;
  NRF_TIMER2->TASKS_STOP = 1;
    
  NRF_TIMER3->TASKS_CLEAR = 1;
  NRF_TIMER2->TASKS_CLEAR = 1;
}

void initTimers(){
  stopTimers(); // Timers must be stopped to change settings
}
  
void initLaserTimer(){
  stopTimers(); // Timers must be stopped to change settings
  
  /* Timer 3 will turn toggle the laser every 50 ms */
  NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
  NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Enabled << TIMER_SHORTS_COMPARE3_CLEAR_Pos; // reset the timer when it reaches 100 ms
  NRF_TIMER3->PRESCALER = 7; // Timer freq = 16MHz/(2^PRESCALE) = 125 kHz
  /*Compare values are set by multiplying timer freq (in kHz) by time (in ms) */
  NRF_TIMER3->CC[0] = 375;  // wait for laser to stabilize, then start sampling                                 125*onDelay/1000
  NRF_TIMER3->CC[1] = 6250;   // laser is on for 50 ms total (t = 50 ms), then turn laser off                   125*laserPeriod 
  NRF_TIMER3->CC[2] = 6563; // wait for laser to stabilize, then start sampling                                 125*(laserPeriod + offDelay/1000)
  NRF_TIMER3->CC[3] = 12500; // laser is off for 50 ms total (t = 100 ms), then turn laser on                   2*125*laserPeriod
}

void initSamplingTimer(int timePerSample){
  stopTimers(); // Timers must be stopped to change settings

  /* Timer 2 will trigger sampling on the SAADC */
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos; // reset the timer when it reaches CC[0]
  NRF_TIMER2->PRESCALER = 4; // Timer freq = 16MHz/(2^PRESCALE) = 1 MHz
  NRF_TIMER2->CC[0] = timePerSample; // sets the value of compare register (also number of us between samples)
}


/*----------------------------------------------------------------------------------------------    
                           Set up pin tasks and events (GPIOTE)
------------------------------------------------------------------------------------------------ */
void initGPIOTE(){
  /* Sets up pin 9 with tasks and events */
  NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)|           // enable tasks
                          (0x1BUL << GPIOTE_CONFIG_PSEL_Pos)|                            // select pin 27 (maps to pin 9 on Nano)
                          (0x0UL << 0xDUL)|                                              // this sets the port to 0
                          (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)| // set up toggle task
                          (GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos);     // pin will be initialized to high
  /* Sets up pin 8 with tasks and events */
  NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)|           // enable tasks
                          (0x15UL << GPIOTE_CONFIG_PSEL_Pos)|                            // select pin 21 (maps to pin 8 on Nano)
                          (0x0UL << 0xDUL)|                                              // this sets the port to 0
                          (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)| // set up toggle task
                          (GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos);     // pin will be initialized to high
}

/*----------------------------------------------------------------------------------------------    
                       Set up (Programmable Peripheral Interconnect) PPI 
------------------------------------------------------------------------------------------------ */
void initPPI(){
  NRF_PPI->CH[0].EEP   = ( uint32_t ) &NRF_TIMER3->EVENTS_COMPARE[0];   // When Timer 3 reaches compare 0
  NRF_PPI->CH[0].TEP   = ( uint32_t ) &NRF_TIMER2->TASKS_START;         // Start Timer 2
  NRF_PPI->FORK[0].TEP = ( uint32_t ) &NRF_SAADC->TASKS_START;          // Start SAADC
  
  NRF_PPI->CH[1].EEP   = ( uint32_t ) &NRF_TIMER3->EVENTS_COMPARE[1];   // When Timer 3 reaches compare 1
  NRF_PPI->CH[1].TEP   = ( uint32_t ) &NRF_GPIOTE->TASKS_SET[0];        // Set pin 9 high (turn laser off)
  NRF_PPI->FORK[1].TEP = ( uint32_t ) &NRF_TIMER2->TASKS_CLEAR;         // Clear Timer 2

  NRF_PPI->CH[2].EEP   = ( uint32_t ) &NRF_TIMER3->EVENTS_COMPARE[2];   // When Timer 3 reaches compare 2
  NRF_PPI->CH[2].TEP   = ( uint32_t ) &NRF_TIMER2->TASKS_START;         // Start Timer 2
  NRF_PPI->FORK[2].TEP = ( uint32_t ) &NRF_SAADC->TASKS_START;          // Start SAADC

  NRF_PPI->CH[3].EEP   = ( uint32_t ) &NRF_TIMER3->EVENTS_COMPARE[3];   // When Timer 3 reaches compare 1
  NRF_PPI->CH[3].TEP   = ( uint32_t ) &NRF_GPIOTE->TASKS_CLR[0];        // Set pin 9 low (turn laser on)
  NRF_PPI->FORK[3].TEP = ( uint32_t ) &NRF_TIMER2->TASKS_CLEAR;         // Clear Timer 2

  NRF_PPI->CH[4].EEP   = ( uint32_t ) &NRF_TIMER2->EVENTS_COMPARE[0];   // When Timer 2 reaches compare 0
  NRF_PPI->CH[4].TEP   = ( uint32_t ) &NRF_SAADC->TASKS_SAMPLE;         // Sample on SAADC (all channels)
  NRF_PPI->FORK[4].TEP = ( uint32_t ) &NRF_GPIOTE->TASKS_OUT[1];        // Toggle pin 8

  NRF_PPI->CHENSET = ( 1UL << 0 )|                                      // Enable channels 0-4
                     ( 1UL << 1 )|
                     ( 1UL << 2 )|
                     ( 1UL << 3 )|
                     ( 1UL << 4 );
}


/*----------------------------------------------------------------------------------------------    
                                Startup Routine
------------------------------------------------------------------------------------------------ */
void startupRoutine( uint16_t& _diagnosticCode, uint32_t& _calibration ){ // passing arguments by reference (&) allows the function to modify the value of the variable
  DateTime startupTime = rtc.now();  
  int startupTemp = HTS_floatToInt(HTS.readTemperature());
  int startupHum = HTS_floatToInt(HTS.readHumidity());
  int alR, alG, alB, ambientLight;
  while ( !APDS.colorAvailable() ){}
  APDS.readColor(alR, alG, alB, ambientLight);
  int32_t refDetDarkNoise;
  int32_t fluorDetDarkNoise;
  int32_t laserIntensity;
  int32_t bkgdFluor;
  int32_t stdFluor;
  int32_t stdRef;

  NRF_GPIOTE->TASKS_SET[0] = 1;               // turn laser off
  delay( 3 );                                 // wait 3 ms
  collectOneCycle(fluorDetDarkNoise, refDetDarkNoise);

  NRF_GPIOTE->TASKS_CLR[0] = 1;               // turn laser on
  delay( 3 );                                 // wait 3 ms for laser to warm up
  collectOneCycle(bkgdFluor, laserIntensity);

  openShutter();
  delay( 3 );                                 // wait 3 ms
  collectOneCycle(stdFluor, stdRef);
  closeShutter();

  NRF_GPIOTE->TASKS_SET[0] = 1;               // turn laser off

  _calibration = (stdRef*standardCorrectionFactor)/stdFluor;

  _diagnosticCode = 0x0000 | ( (startupTemp > tempLimitH) << tempCodePos ) | // temp within specs
                             ( (startupHum > humLimitH) << humCodePos  ) | // hum within specs
                             ( (refDetDarkNoise > refNoiseLimitH) << rddCodePos  ) | // ref detector dark noise OK
                             ( (fluorDetDarkNoise > fluorNoiseLimitH) << fddCodePos  ) | // fluor detector dark noise OK
                             ( (laserIntensity < laserLimitL) << laserCodePos )| // laser OK
                             ( (laserIntensity > laserLimitH) << laserCodePos )| // laser OK
                             ( (bkgdFluor > bkgdLimitH) << bkgdCodePos )| // background fluorescence OK
                             ( (ambientLight > lightLimitH) << lightCodePos )| // ambient light OK
                             ( (_calibration < calLimitL)   << calCodePos  ) ; // calibration OK

  char startupFile[] = "startup.txt";
  
  if(!file.open(startupFile, O_CREAT | O_TRUNC | O_WRITE)){     // create file
    if(!sd.begin(SS)){
      errorFlag = true;
    }
  }
    
  file.println( F("Startup and Calibration") );
  file.println( startupTime.timestamp(DateTime::TIMESTAMP_FULL) );
  file.print( F("Sensor internal temperature: ") );
  file.print( startupTemp );
  file.println( F(" F") );
  file.print( F("Sensor internal humidity: ") );
  file.println( startupHum );
  file.print( F("Reference detector dark noise: ") );
  file.println( refDetDarkNoise );
  file.print( F("Fluorescence detector dark noise: ") );
  file.println( fluorDetDarkNoise );
  file.print( F("Laser intensity: ") );
  file.println( laserIntensity );
  file.print( F("Background fluorescence: ") );
  file.println( bkgdFluor );
  file.print( F("Ambient light: ") );
  file.println( ambientLight );
  file.print( F("Calibration: ") );
  file.print( _calibration );
  file.println( F("(equivalent to ?? ucg/L chl a standard)" ) );
  file.print( F("Diagnostic code: ") );
  file.println( _diagnosticCode );
  
  file.close();
}

void collectOneCycle(int32_t& fluorVal, int32_t& refVal){
  adcFlag = 0;                                          // clear ADC flag
  NRF_SAADC->TASKS_START = 1;                           // start SAADC
  NRF_TIMER2->TASKS_START = 1;                          // start Timer 2 to begin sampling SAADC
  while( !adcFlag ){}                                   // wait for Result buffer to fill (128 samples each channel)

  fluorVal = 0;
  refVal = 0;

  for( int i=0; i<ADC_BUFFER_SIZE; i+=NUM_CHANNELS ){
    fluorVal += adcBuffer[i];                           // sum results from channel 0
    refVal += adcBuffer[i+1];                           // sum results from channel 1
  }
}

void openShutter(){
  digitalWrite(shutterEN, HIGH);              // enable shutter
  digitalWrite(shutterPos, HIGH);             // open shutter
  digitalWrite(shutterNeg, LOW);
  delay(200);
  digitalWrite(shutterEN, LOW);               // disable shutter
  digitalWrite(shutterPos, LOW);          
}

void closeShutter(){  
  digitalWrite(shutterEN, HIGH);              // enable shutter
  digitalWrite(shutterPos, LOW);              // close shutter
  digitalWrite(shutterNeg, HIGH); 
  delay(200);
  digitalWrite(shutterEN, LOW);               // disable shutter
  digitalWrite(shutterNeg, LOW); 
}

int HTS_floatToInt(float HTS_float){
  float decimalShift = 100*HTS_float;
  int HTS_int = static_cast<int>(decimalShift);
  return HTS_int;
}

void startSensor(){
  adcFlag = 0;                              // clear adcFlag
  fluorOn = fluorOff = refOn = refOff = 0;  // reset all variables
  NRF_GPIOTE->TASKS_CLR[0] = 1;             // turn laser on
  active = true;
}


void shutdownSensor(){
  stopTimers();                             // stop the Timers
  NRF_SAADC->TASKS_STOP = 1;                // stop the SAADC
  NRF_GPIOTE->TASKS_SET[0] = 1;             // turn laser off
  file.close();                             // sync and close file to save it on the SD card
  active = false;
}

void getTempHum(){
  temperature.data = HTS.readTemperature(); // read temp and humidity data
  humidity.data = HTS.readHumidity();
  tempPub.publish( &temperature );          // publish temp on the "temp" ROS topic
  humPub.publish( &humidity );              // publish humidity on the "humidity" ROS topic
}

void createNewFile(){
  char fileName[] = "data00.txt";                            // generate new file name
  uint8_t fileNumber = 0;
  while(sd.exists(fileName)){                                // if file already exists
    fileName[4] = fileNumber/10 + '0';                       // sets the 10's place
    fileName[5] = fileNumber%10 + '0';                       // sets the 1's place
    fileNumber++;                                            // try the next number
  }  
  if(!file.open(fileName, O_CREAT | O_TRUNC | O_WRITE)){     // create file
    Serial.println( F("Error opening file") );
    if(!sd.begin(SS)){
      errorFlag = true;
    }
  }else{
    Serial.println( F("File openned successfully") );
  }
}

void getData( int32_t& fluorData, int32_t& refData ){
  for( int i=0; i<ADC_BUFFER_SIZE; i+=NUM_CHANNELS ){
    fluorData += adcBuffer[i];        // sum results from channel 0
    refData += adcBuffer[i+1];        // sum results from channel 1
  }
}

void reportResults( int32_t _fluorOn, int32_t _fluorOff, int32_t _refOn, int32_t _refOff ){
  int32_t fluorResult = (_fluorOn - _fluorOff);       // calculate results
  int32_t refResult = (_refOn - _refOff);       
  adcFlag = 0;                                        // reset flag

  fluorMsg.data = fluorResult;                        // update fluorMsg ROS message
  refMsg.data = refResult;                            // update refMsg ROS message
  fluorPub.publish( &fluorMsg );                      // publish fluor on the "fluor_data" ROS topic
  refPub.publish( &refMsg );                          // publish ref on the "ref_data" ROS topic
}
