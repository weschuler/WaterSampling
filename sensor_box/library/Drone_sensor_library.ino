/*
  This sketch is for testing and debugging a library of functions for a sensor based on the Arduino Nano BLE sense
*/

#include "Drone_sensor.h"
#include <ros.h>                  // ROS main library and data type libraries
#include <ros/time.h>
#include <std_msgs/Int16.h>       
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>


//#define DEBUG 1     // code scaffold for enabling/disabling debugging output on the serial monitor
#define ROS_TIMEOUT_DELAY 100

Drone_sensor sensor;        // set up sensor with 10 Hz report freq

Drone_sensor::ADC_channel<128> fluorescence(8); // create fluorescence channel object with oversample = 8
Drone_sensor::ADC_channel<128> reference(8); // create reference channel object with oversample = 8
Drone_sensor::ADC_channel<1024> scattering(1); // scattering channel isn't oversampled, so buffer is 8x larger

ros::NodeHandle nh;               // create node handle object for ROS

void messageCb(const std_msgs::Int16& mode) {       // Callback function for subscriber input on ROS
    sensor.setMode(mode.data);
}

ros::Subscriber<std_msgs::Int16> mode("sensor/mode", messageCb );    // Declare subscriber to messages on "mode" topic, trigger callback function

std_msgs::Float32 fluorMsg;                                     // Set up data type for fluorescence data message
std_msgs::Float32 refMsg;                                       // Set up data type for reference data message
std_msgs::Float32 temperature;                                // Set up data type for temp data message
std_msgs::Float32 humidity;                                   // Set up data type for humidity data message
std_msgs::String myErrorMsg;                                       // Set up error message
std_msgs::UInt32 calibrationMsg;
std_msgs::Int16 diagnostic;

ros::Publisher fluorPub("sensor/fluor", &fluorMsg);             // Declare publisher of fluorescence data message on "fluor_data" topic
ros::Publisher refPub("sensor/ref", &refMsg);                   // Declare publisher of reference data message on "ref_data" topic
ros::Publisher tempPub("sensor/temp", &temperature);                 // Declare publisher of temp data message on "temp" topic
ros::Publisher humPub("sensor/humidity", &humidity);                 // Declare publisher of humidity data message on "humidity" topic
ros::Publisher errorPub("sensor/error", &myErrorMsg);                     // Set up publisher of error topic
ros::Publisher calibrationPub("sensor/calibration", &calibrationMsg);
ros::Publisher diagnosticPub("sensor/diagnostic", &diagnostic);

unsigned long rosTimeout = ROS_TIMEOUT_DELAY;
unsigned long tempTimeout = 1000;

//------------------------------------------------------------------------------
// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = sensor.rtc.now();

  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(now.year(), now.month(), now.day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(now.hour(), now.minute(), now.second());

  // Return low time bits in units of 10 ms, 0 <= ms10 <= 199.
  *ms10 = now.second() & 1 ? 100 : 0;
}

void setup() {
    fluorescence.config.pin_p = NRF_SAADC_INPUT_AIN2;       // set input for fluorescence channel (Nano pin A0)
    fluorescence.config.gain = NRF_SAADC_GAIN1;             // set gain for fluorescence channel
    reference.config.pin_p = NRF_SAADC_INPUT_AIN6;          // set input for reference channel (Nano pin A2)
    reference.config.gain = NRF_SAADC_GAIN1_4;              // set gain for reference channel
    scattering.config.pin_p = NRF_SAADC_INPUT_AIN5;          // set input for scattering channel (Nano pin A3)
    scattering.config.gain = NRF_SAADC_GAIN1_4;              // set gain for scattering channel

#ifndef DEBUG
    nh.initNode();                 // Initialize node handle
    nh.advertise(fluorPub);        // Start advertising/publishing on "fluor_data" topic
    nh.advertise(refPub);          // Start advertising/publishing on "ref_data" topic
    nh.advertise(tempPub);         // Start advertising/publishing on "temp" topic
    nh.advertise(humPub);          // Start advertising/publishing on "humidity" topic
    nh.advertise(errorPub);        // Advertise error topic
    nh.advertise(calibrationPub);
    nh.advertise(diagnosticPub);
    nh.subscribe(mode);            // Initialize subscriber to messages on "record" topic
#endif // !DEBUG

    // Set file timestamp callback
    FsDateTime::setCallback(dateTime);

#ifdef DEBUG
	Serial.begin(115200);    // use Serial output for debugging
	while (!Serial) {}
#endif // DEBUG
	if (sensor.init()) { // initialize sensor
#ifdef DEBUG
		Serial.println("Sensor.init success");
	}
	else {
		Serial.println("Sensor.init failed");
#endif // DEBUG
	}
    sensor.setMode(2);
}


void loop() {
	switch (sensor.getMode()) {
    // Idle Mode    
    case 0: {
        if (sensor.isActive()) {
            sensor.shutdown();
#ifdef DEBUG
            Serial.println(F("Shutting down"));
#endif // DEBUG
        }
#ifndef DEBUG
        if (sensor._errorCase) {
            myErrorMsg.data = sensor.readErrors();
            errorPub.publish(&myErrorMsg);
        }
#endif // DEBUG
        if (millis() > rosTimeout) {
#ifdef DEBUG
            if (sensor._errorCase != 0) {
                Serial.println(sensor._errorCase);
                Serial.println(sensor.readErrors());
                Serial.println(sensor._errorCase);
            }

#endif // DEBUG
#ifndef DEBUG
            nh.spinOnce();
#endif // !DEBUG
            rosTimeout = millis() + ROS_TIMEOUT_DELAY;
        }
        if (millis() > tempTimeout) {    // once per second, the sensor will read new temp and hum values and store them to the SD card
            sensor.getHTS();
            temperature.data = sensor.HTSdata.temperature;
            humidity.data = sensor.HTSdata.humidity;
#ifndef DEBUG
            tempPub.publish(&temperature);
            humPub.publish(&humidity);
#endif // !DEBUG
#ifdef DEBUG 
            Serial.print("Temp: ");
            Serial.println(sensor.HTSdata.temperature);
            Serial.print("Humidity: ");
            Serial.println(sensor.HTSdata.humidity);
#endif // DEBUG

            tempTimeout = millis() + 1000;
        }
    }
        break;
    // Record Mode (data)
    case 1: {     
        Drone_sensor::ADC<2048> adc;
        adc.enableChannel(&fluorescence);
        adc.enableChannel(&reference);
        sensor.initADC(&adc);
#ifdef DEBUG
        Serial.print(adc._numChannels);
        Serial.println(F("channels enabled"));
        Serial.print(fluorescence.uVperLSB);
        Serial.println(F(" uV/LSB (fluorescence)"));
        Serial.print(reference.uVperLSB);
        Serial.println(F(" uV/LSB (reference)"));
        Serial.println(F("Creating new file"));
#endif // DEBUG
        sensor.newFile();
        sensor.start();                 // laser is on
        while (sensor.getMode() == 1) {
            while (!sensor.dataReady()) {}
            sensor.laser(false);        // laser is off
#ifdef DEBUG
            Serial.println(F("Getting on data"));
#endif // DEBUG
            fluorMsg.data = adc.getData(&fluorescence);
            refMsg.data = adc.getData(&reference);

            sensor.start();
            sensor.writeFile(fluorescence);
            sensor.writeFile(reference);

            while (!sensor.dataReady()) {}
            sensor.laser(true);        // laser is on
#ifdef DEBUG
            Serial.println(F("Getting off data"));
#endif // DEBUG
            fluorMsg.data -= adc.getData(&fluorescence);
            refMsg.data -= adc.getData(&reference);

            sensor.start();            // start next collection

            sensor.writeFile(fluorescence);
            calibrationMsg.data = sensor.writeFile(reference);

#ifndef DEBUG
            fluorPub.publish(&fluorMsg);    // publish fluor on the "fluor_data" ROS topic
            refPub.publish(&refMsg);        // publish ref on the "ref_data" ROS topic
            nh.spinOnce();                  // update ROS
#endif // !DEBUG

#ifdef DEBUG
            Serial.print("Write counter: ");
            Serial.println(calibrationMsg.data);
            Serial.print(fluorMsg.data);
            Serial.print("\t");
            Serial.println(refMsg.data);

            if (Serial.available()) {
                sensor.setMode(Serial.parseInt());
            }
#endif // DEBUG
        }
        sensor.shutdown();
#ifdef DEBUG
        Serial.println("Shutting down");
#endif // DEBUG
    }
    	break;
    // Calibrate (cali)
    case 2: {
        if (!sensor.newFolder()) {
            break;
        }
        if (!sensor.newFile()) {
            break;
        }

        ros::Time ROStimestamp = nh.now();
        DateTime startupTimeROS = DateTime(ROStimestamp.sec);
        DateTime startupTimeRTC = sensor.rtc.now();

        Drone_sensor::ADC_channel<128> _fluorescence(8); // create fluorescence channel object with oversample = 8
        Drone_sensor::ADC_channel<128> _reference(8); // create reference channel object with oversample = 8

        Drone_sensor::ADC<2048> adc;

        adc.enableChannel(&fluorescence);
        adc.enableChannel(&reference);
        sensor.initADC(&adc);

        float bkgdFluor = 0;
        float laserIntensity = 0;
        float blank[100] = { 0 };
        float sse = 0;
        float blankNoise = 0;

        for (int i = 0; i < 100; i++) {
            sensor.start();
            while (!sensor.dataReady()) {}
            sensor.laser(false);
            blank[i] = adc.getData(&fluorescence);
            laserIntensity += adc.getData(&reference);

            sensor.start();
            while (!sensor.dataReady()) {}
            sensor.laser(true);
            blank[i] -= adc.getData(&fluorescence);
            bkgdFluor += blank[i];
            laserIntensity -= adc.getData(&reference);
#ifndef DEBUG
            if (i % 10 == 0) {
                nh.spinOnce();
            }
#endif // !DEBUG
        }

        bkgdFluor = bkgdFluor / 100.0;
        laserIntensity = laserIntensity / 100.0;

        for (int i = 0; i < 100; i++) {
            sse += sq(blank[i] - bkgdFluor);
        }
        blankNoise = sqrt(sse / 100.0);

        sensor.file.println(F("Startup and Calibration"));
        sensor.file.print(F("RTC timestamp: "));
        sensor.file.println(startupTimeRTC.timestamp(DateTime::TIMESTAMP_FULL));
        sensor.file.print(F("ROS timestamp: "));
        sensor.file.println(startupTimeROS.timestamp(DateTime::TIMESTAMP_FULL));
        sensor.file.print(F("Sensor internal temperature: "));
        sensor.file.print(HTS.readTemperature());
        sensor.file.println(F(" C"));
        sensor.file.print(F("Sensor internal humidity: "));
        sensor.file.println(HTS.readHumidity());
        sensor.file.print(F("Laser intensity (uV): "));
        sensor.file.println(laserIntensity);
        sensor.file.print(F("Background fluorescence (uV): "));
        sensor.file.println(bkgdFluor);
        sensor.file.print(F("Fluorescence detector blank noise (uV): "));
        sensor.file.println(blankNoise);
        //sensor.file.print(F("Ambient light: "));
        //sensor.file.println(ambientLight);
        //sensor.file.print(F("Calibration: "));
        //sensor.file.print(calibration);
        //sensor.file.println(F("(equivalent to ?? ucg/L chl a standard)"));
        //sensor.file.print(F("Diagnostic code: "));
        //sensor.file.println(diagnosticCode);

        sensor.shutdown();
        sensor.setMode(0);
    }
        break;
    // Laser On
    case 3: {
        sensor.laser(true);
        sensor.setMode(0);
    }
        break;
    // Laser Off
    case 4: {
        sensor.laser(false);
        sensor.setMode(0);
    }
        break;
    // Open shutter
    case 5: {
        sensor.shutter(true);
        sensor.setMode(0);
    }
        break;
    // Close shutter
    case 6: {
        sensor.setMode(0);
        sensor.shutter(false);
    }
        break;
    // Test mode (test)
    case 7: {
        Drone_sensor::ADC<2048> adc;
        adc.enableChannel(&fluorescence);
        adc.enableChannel(&reference);
        sensor.initADC(&adc);
#ifdef DEBUG
        Serial.print(adc._numChannels);
        Serial.println(F("channels enabled"));
        Serial.print(fluorescence.uVperLSB);
        Serial.println(F(" uV/LSB (fluorescence)"));
        Serial.print(reference.uVperLSB);
        Serial.println(F(" uV/LSB (reference)"));
        Serial.println(F("Creating new file"));
#endif // DEBUG
        sensor.newFile();
        sensor.start();  

        while (sensor.getMode() == 7) {
            while (!sensor.dataReady()) {}
#ifdef DEBUG
            Serial.println(F("Getting data"));
#endif // DEBUG
            fluorMsg.data = adc.getData(&fluorescence);
            refMsg.data = adc.getData(&reference);

            sensor.start();                             // laser is on
            sensor.writeFile(fluorescence);
            sensor.writeFile(reference);

#ifndef DEBUG
            fluorPub.publish(&fluorMsg);    // publish fluor on the "fluor_data" ROS topic
            refPub.publish(&refMsg);        // publish ref on the "ref_data" ROS topic
            nh.spinOnce();                  // update ROS
#endif // !DEBUG

#ifdef DEBUG
            Serial.print(fluorMsg.data);
            Serial.print("\t");
            Serial.println(refMsg.data);

            if (Serial.available()) {
                sensor.setMode(Serial.parseInt());
            }
#endif // DEBUG
            switch (sensor.getMode()) {
            case 3:
                sensor.laser(true);
                sensor.setMode(7);
                break;
            case 4:
                sensor.laser(false);
                sensor.setMode(7);
                break;
            case 5:
                sensor.shutter(true);
                sensor.setMode(7);
                break;
            case 6:
                sensor.shutter(false);
                sensor.setMode(7);
                break;
            default:
                break;
            }
        }
        sensor.shutdown();
    }
        break;
    // IMU vibration mode (vibe)
    case 8: {
        sensor.newFile();
        while (sensor.getMode() == 8) {
            sensor.getIMU();
            sensor.writeFile(sensor.IMUdata);
#ifndef DEBUG
            if (millis() > rosTimeout) {
                nh.spinOnce();
                rosTimeout = millis() + ROS_TIMEOUT_DELAY;
            }
#endif // !DEBUG

#ifdef DEBUG
            Serial.println("AccX\tAccY\tAccZ\tGyrX\tGyrY\tGyrZ");
            Serial.print(sensor.IMUdata[0]);
            Serial.print("\t");
            Serial.print(sensor.IMUdata[1]);
            Serial.print("\t");
            Serial.print(sensor.IMUdata[2]);
            Serial.print("\t");
            Serial.print(sensor.IMUdata[3]);
            Serial.print("\t");
            Serial.print(sensor.IMUdata[4]);
            Serial.print("\t");
            Serial.println(sensor.IMUdata[5]);

            if (Serial.available()) {
                sensor.setMode(Serial.parseInt());
            }
#endif // DEBUG
        }
        sensor.shutdown();
    }
            break;
    // Microphone mode (audi)
    case 9: {
        sensor.newFile();
        PDM.onReceive(onPDMdata);
        PDM.begin(1, 16000); // init microphone, 1 channel at 16 kHz
        while (sensor.getMode() == 9) {
            if (sensor.PDMdata.flag) {
                sensor.writeFile(sensor.PDMdata);
                sensor.PDMdata.flag = false;
            }
            if (millis() > rosTimeout) {
#ifdef DEBUG
                Serial.println(F("Recording audio"));
#endif // DEBUG
#ifndef DEBUG
                nh.spinOnce();
#endif // !DEBUG
                rosTimeout = millis() + ROS_TIMEOUT_DELAY;
        }
#ifdef DEBUG
            if (Serial.available()) {
                sensor.setMode(Serial.parseInt());
            }
#endif // DEBUG
        }
        sensor.shutdown();
    }
            break;
    // HTS mode (clim)
    case 10: {
        sensor.newFile();
        while (sensor.getMode() == 10) {
            if (millis() > tempTimeout) {    // once per second, the sensor will read new temp and hum values and store them to the SD card
                sensor.getHTS();
                sensor.writeFile(sensor.HTSdata);
                tempTimeout = millis() + 1000;
            }
            if (millis() > rosTimeout) {
#ifdef DEBUG
                Serial.println(F("Recording temp and humidity"));
#endif // DEBUG
#ifndef DEBUG
                nh.spinOnce();
#endif // !DEBUG
                rosTimeout = millis() + ROS_TIMEOUT_DELAY;
            }
#ifdef DEBUG
            if (Serial.available()) {
                sensor.setMode(Serial.parseInt());
            }
#endif // DEBUG
        }
        sensor.shutdown();
    }
        break;
    // Scattering only (scat)
    case 11: {
        Drone_sensor::ADC<1024> adc;
        adc.enableChannel(&scattering);
        sensor.initADC(&adc);
#ifdef DEBUG
        Serial.println(F("Creating new file"));
#endif // DEBUG
        sensor.newFile();
        sensor.start();
        while (sensor.getMode() == 11) {
            while (!sensor.dataReady()) {}
#ifdef DEBUG
            Serial.println(F("Getting data"));
#endif // DEBUG
            adc.getData(&scattering);

            sensor.start();                             // laser is on
            sensor.writeFile(scattering);

#ifndef DEBUG
            if (millis() > rosTimeout) {
                nh.spinOnce();                          // update ROS
                rosTimeout = millis() + ROS_TIMEOUT_DELAY;
            }               
#endif // !DEBUG

#ifdef DEBUG
            if (Serial.available()) {
                sensor.setMode(Serial.parseInt());
            }
#endif // DEBUG
        }
        sensor.shutdown();
        }
        break;
    // All detectors (adcb)
    case 12: {
        Drone_sensor::ADC<3072> adc;
        adc.enableChannel(&fluorescence);
        adc.enableChannel(&reference);
        adc.enableChannel(&scattering);
        sensor.initADC(&adc);
#ifdef DEBUG
        Serial.print(adc._numChannels);
        Serial.println(F("channels enabled"));
        Serial.print(fluorescence.uVperLSB);
        Serial.println(F(" uV/LSB (fluorescence)"));
        Serial.print(reference.uVperLSB);
        Serial.println(F(" uV/LSB (reference)"));
        Serial.println(F("Creating new file"));
#endif // DEBUG
        sensor.newFile();
        sensor.start();                 // laser is on
        while (sensor.getMode() == 12) {
            while (!sensor.dataReady()) {}  // wait for data buffer to fill
            sensor.laser(false);            // turn off laser
#ifdef DEBUG
            Serial.println(F("Getting on data"));
#endif // DEBUG
            fluorMsg.data = adc.getData(&fluorescence);
            refMsg.data = adc.getData(&reference);
            adc.getData(&scattering);

            sensor.start();                 // start next collection
            sensor.writeFile(fluorescence); // write data to SD card
            sensor.writeFile(reference);
            sensor.writeFile(scattering);
            
            while (!sensor.dataReady()) {}  // wait for data buffer to fill
            sensor.laser(true);             // turn on laser
#ifdef DEBUG
            Serial.println(F("Getting off data"));
#endif // DEBUG
            fluorMsg.data -= adc.getData(&fluorescence);
            refMsg.data -= adc.getData(&reference);
            adc.getData(&scattering);

            sensor.start();                 // start next collection
            sensor.writeFile(fluorescence); // write data to SD card
            sensor.writeFile(reference);
            sensor.writeFile(scattering);

#ifndef DEBUG
            fluorPub.publish(&fluorMsg);    // publish fluor on the "fluor_data" ROS topic
            refPub.publish(&refMsg);        // publish ref on the "ref_data" ROS topic
            nh.spinOnce();                  // update ROS
#endif // !DEBUG

#ifdef DEBUG
            Serial.print(fluorMsg.data);
            Serial.print("\t");
            Serial.println(refMsg.data);

            if (Serial.available()) {
                sensor.setMode(Serial.parseInt());
            }
#endif // DEBUG
        }
        sensor.shutdown();
    }
        break;
    case 13: {
        Drone_sensor::ADC<1024> adc;
        adc.enableChannel(&fluorescence);
        sensor.initADC(&adc);
#ifdef DEBUG
        Serial.print(adc._numChannels);
        Serial.println(F("channels enabled"));
        Serial.print(fluorescence.uVperLSB);
        Serial.println(F(" uV/LSB (fluorescence)"));
        Serial.println(F("Creating new file"));
#endif // DEBUG
        sensor.newFile();
        sensor.start();                 // laser is on
        while (sensor.getMode() == 13) {
            while (!sensor.dataReady()) {}  // wait for data buffer to fill
#ifdef DEBUG
            Serial.println(F("Getting data"));
#endif // DEBUG
            fluorMsg.data = adc.getData(&fluorescence);
            
            sensor.start();                 // start next collection
            sensor.writeFile(fluorescence); // write data to SD card
  
#ifndef DEBUG
            fluorPub.publish(&fluorMsg);    // publish fluor on the "fluor_data" ROS topic
            nh.spinOnce();                  // update ROS
#endif // !DEBUG

#ifdef DEBUG
            Serial.print(fluorMsg.data);

            if (Serial.available()) {
                sensor.setMode(Serial.parseInt());
            }
#endif // DEBUG
        }
        sensor.shutdown();
    }
        break;
    default: {
        sensor.setMode(0);
    }
  		break;
	}
#ifdef DEBUG
	if (Serial.available()) {
        sensor.setMode(Serial.parseInt());
	}
#endif // DEBUG
#ifndef DEBUG
    if (millis() > rosTimeout) {
        nh.spinOnce();
        rosTimeout = millis() + ROS_TIMEOUT_DELAY;
    }
#endif // !DEBUG
}


/* callback for microphone */
void onPDMdata() {
    // Query the number of available bytes
    sensor.PDMdata.bytesAvailable = PDM.available();
    PDM.read(sensor.PDMdata.sampleBuffer, sensor.PDMdata.bytesAvailable);
    sensor.PDMdata.flag = true;
}
