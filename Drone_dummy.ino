/*
  This code simulates the function of the drone-mounted chlorophyll fluorescence sensor.
  
  The circuit: Arduino Nano; L293D dual half h-bridge for switching laser and shutter using pins 4-9; 
  differential input channel on A0/A1 for the fluorescence detector; differential input channel on A2/A3
  for the reference detector; communication with Raspberry Pi via ROS on UART (TX/RX).

  Pin 8 and 9 are controlled by tasks and events through the PPI. Pin 9 turns the laser on and off;
  pin 8 is just an indicator.

 
  In recording mode, the Arduino produces a 1 Hz sawtooth wave for both the reference and fluorescence channels.
  This data is published to ROS topics at 10 Hz which are monitored by the Pi.
  
*/

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "mbed.h"
#include <Arduino_HTS221.h>       // library for temp and humidity sensor


ros::NodeHandle nh;               // create node handle object for ROS


#define ADC_BUFFER_SIZE   256     // the number of 16 bit samples to store in the data buffer
#define max_file_size  100        // the max file size in kilobytes
#define NUM_CHANNELS 2            // 2 ADC channels (fluor and ref)

#define reportFreq  10            // sets frequency of updates to ROS in Hz

#define laserPeriod 50            // laser on (or off) time = half of the laser period in ms
#define onDelay 3000              // time it takes for laser to fully turn on in us
#define offDelay 2500             // time it takes for laser to fully turn off in us

const int numSamples = ADC_BUFFER_SIZE/NUM_CHANNELS;       // number of samples per sampling period (laser on or off)
const int sampleTime = (1000*(laserPeriod-3))/numSamples;  // amount of time per sample in us (laser period padded with 3 ms delay)

volatile nrf_saadc_value_t adcBuffer[ADC_BUFFER_SIZE] = {0};   // result buffer, initiallized to 0
volatile int16_t * bufferPtr = adcBuffer;                      // pointer to the result buffer


volatile int adcFlag = 0;         // flag set by SAADC ISR when buffer is full
 
volatile int16_t sensorMode = 0;  // 0 = idle, 1 = recording, 2 = calibrate
bool active = false;              // true while sensor is running

int32_t fluorOn, refOn;           // variables for recording data
uint32_t lastReading;             // used to time humidity and temp readings
bool readingReady;                // true when a new humidity/temp reading is ready

int sawtooth = 0;                 // sawtooth dummy data
int stepSize = -10000;            // sawtooth step size
int dummyMin = 0;
int dummyMax = 500000;

const int shutterEN = 4;          // pin 4 is the enable pin for the shutter
const int shutterPos = 5;
const int shutterNeg = 6; 


/*--------------------------------------------------------------------------------------------- 
              Callback function for subscriber input on ROS
 ---------------------------------------------------------------------------------------------*/

void messageCb( const std_msgs::Int16& mode){
  sensorMode = mode.data;                                     // Set flag to start or stop recording (interrupts the while(recording) loop
}

ros::Subscriber<std_msgs::Int16> mode("mode", messageCb );    // Declare subscriber to messages on "mode" topic, trigger callback function

std_msgs::Int32 fluorMsg;                                     // Set up data type for fluorescence data message
std_msgs::Int32 refMsg;                                       // Set up data type for reference data message
std_msgs::Float32 temperature;                                // Set up data type for temp data message
std_msgs::Float32 humidity;                                   // Set up data type for humidity data message

ros::Publisher fluorPub("fluor_data", &fluorMsg);             // Declare publisher of fluorescence data message on "fluor_data" topic
ros::Publisher refPub("ref_data", &refMsg);                   // Declare publisher of reference data message on "ref_data" topic
ros::Publisher tempPub("temp", &temperature);                 // Declare publisher of temp data message on "temp" topic
ros::Publisher humPub("humidity", &humidity);                 // Declare publisher of humidity data message on "humidity" topic


void setup(){
  pinMode(shutterEN, OUTPUT);
  pinMode(shutterPos, OUTPUT);
  pinMode(shutterNeg, OUTPUT);

  HTS.begin();                   // Initialize temp and humidity sensor

  nh.initNode();                 // Initialize node handle
  nh.advertise(fluorPub);        // Start advertising/publishing on "fluor_data" topic
  nh.advertise(refPub);          // Start advertising/publishing on "ref_data" topic
  nh.advertise(tempPub);         // Start advertising/publishing on "temp" topic
  nh.advertise(humPub);          // Start advertising/publishing on "humidity" topic
  nh.subscribe(mode);          // Initialize subscriber to messages on "mode" topic

  initADC();
  initTimers();
  initPPI();
}



void loop() { 
  
/*---------------------------------------------------------------------------------------------    
                                        Idle Mode
----------------------------------------------------------------------------------------------- */
  while( sensorMode == 0 ){                     // if Pi sets the "mode" flag on ROS to 0 (Idle mode)
    if( active ){                               // if the sensor is still active
      stopTimers();                             // stop the Timers
      NRF_SAADC->TASKS_STOP = 1;                // stop the SAADC
      NRF_GPIOTE->TASKS_SET[0];                 // turn laser off
      active = false;
    }
    
    if( millis() > (lastReading + 5000) ){      // if it has been more than 5 seconds since last reading 
      temperature.data = HTS.readTemperature(); // read temp and humidity data
      humidity.data = HTS.readHumidity();
      lastReading = millis();
      readingReady = true;                      // data is ready to publish
    }
    if( readingReady ){                         // if there is temp and humidity data to publish 
      tempPub.publish( &temperature );          // publish temp on the "temp" ROS topic
      humPub.publish( &humidity );              // publish humidity on the "humidity" ROS topic
      readingReady = false;
    }
    nh.spinOnce();                              // update ROS node
  
  } // end of Idle Mode
  
 

/*---------------------------------------------------------------------------------------------    
                                Record Mode
----------------------------------------------------------------------------------------------- */
  while( sensorMode == 1 ){                     // if Pi sets the "mode" flag on ROS to 1 (Record mode)

    fluorMsg.data = sawtooth;                   // update fluorMsg ROS message with dummy data
    refMsg.data = sawtooth/2;                   // update refMsg ROS message with dummy data
    fluorPub.publish( &fluorMsg );              // publish fluor on the "fluor_data" ROS topic
    refPub.publish( &refMsg );                  // publish ref on the "ref_data" ROS topic

    if( sawtooth == dummyMin || sawtooth == dummyMax ){ //when the wave reaches the max or min
      stepSize = -stepSize;                             // turn around
    }
    sawtooth += stepSize;                       // increment dummy data
        
    delay(1000/reportFreq);                      // this sets the update frequency
    
    nh.spinOnce();                              // update ROS node
  
  } // end of Record mode


/*----------------------------------------------------------------------------------------------    
                                Calibrate Mode
------------------------------------------------------------------------------------------------ */
  while( sensorMode == 2 ){                     // if Pi sets the "mode" flag on ROS to 2 (Calibrate mode)
    digitalWrite(shutterEN, HIGH);              // enable shutter
    digitalWrite(shutterPos, HIGH);             // open shutter
    digitalWrite(shutterNeg, LOW);
    digitalWrite(shutterEN, HIGH);              // disable shutter

    NRF_GPIOTE->TASKS_CLR[0];                   // turn laser on
    delay( 3 );                                 // wait 3 ms for laser to warm up
    
    adcFlag = 0;                                // clear ADC flag
    NRF_SAADC->TASKS_START = 1;                 // start SAADC
    NRF_TIMER2->TASKS_START = 1;                // start Timer 2 to begin sampling SAADC
    while( !adcFlag ){}                         // wait for Result buffer to fill (128 samples each channel)

    fluorOn = refOn = 0;                        // clear variables 
          
    for( int i=0; i<ADC_BUFFER_SIZE; i+=NUM_CHANNELS ){
      fluorOn += adcBuffer[i];                  // sum results from channel 0
      refOn += adcBuffer[i+1];                  // sum results from channel 1
    }

    fluorMsg.data = fluorOn;                    // update fluorMsg ROS message
    refMsg.data = refOn;                        // update refMsg ROS message
    fluorPub.publish( &fluorMsg );              // publish fluor on the "fluor_data" ROS topic
    refPub.publish( &refMsg );                  // publish ref on the "ref_data" ROS topic

    digitalWrite(shutterEN, HIGH);              // enable shutter
    digitalWrite(shutterPos, LOW);              // close shutter
    digitalWrite(shutterNeg, HIGH); 
    digitalWrite(shutterEN, HIGH);              // disable shutter

    nh.spinOnce();                              // update ROS node

    sensorMode = 0;                             // go to Idle mode
    
  } // end of Calibrate mode

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
    settings0.reference  = NRF_SAADC_REFERENCE_VDD4 ;     // VDD/4 Reference
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
    settings1.pin_p      = NRF_SAADC_INPUT_AIN6 ;         // Nano A2
    settings1.pin_n      = NRF_SAADC_INPUT_AIN5 ;         // Nano A3
      
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
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->TASKS_CLEAR = 1;
}

void initTimers(){
  stopTimers(); // Timers must be stopped to change settings
  
  /* Timer 2 will trigger sampling on the SAADC */
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos; // reset the timer when it reaches CC[0]
  NRF_TIMER2->PRESCALER = 4; // Timer freq = 16MHz/(2^PRESCALE) = 1 MHz
  NRF_TIMER2->CC[0] = sampleTime; // sample every 368 us  
}



/*----------------------------------------------------------------------------------------------    
                       Set up (Programmable Peripheral Interconnect) PPI 
------------------------------------------------------------------------------------------------ */
void initPPI(){

  NRF_PPI->CH[4].EEP   = ( uint32_t ) &NRF_TIMER2->EVENTS_COMPARE[0];   // When Timer 2 reaches compare 0
  NRF_PPI->CH[4].TEP   = ( uint32_t ) &NRF_SAADC->TASKS_SAMPLE;         // Sample on SAADC (all channels)

  NRF_PPI->CHENSET = ( 1UL << 4 );                                      // Enable channel 4
                   
}
