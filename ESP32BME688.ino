/*
 * This sketch is a branc of my PubSubWeather sketch.
 * This sketch will use a AHT20/BMP280 combination sensor to show temperature, pressure, and humidity.
 * The ESP-32 SDA pin is GPIO21, and SCL is GPIO22.
 */
#include "WiFi.h"						// This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <Adafruit_Sensor.h>		// Adafruit Unified Sensor Driver.  https://github.com/adafruit/Adafruit_Sensor
#include "Adafruit_BME680.h"		// Adafruit BME680 (and BME688) library.  https://github.com/adafruit/Adafruit_BME680
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.
#include <Adafruit_NeoPixel.h>	// The Adafruit NeoPixel library to drive the RGB LED on the QT Py.	https://github.com/adafruit/Adafruit_NeoPixel


// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS        1
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA ( 1025.0 )


/**
 * Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
//const char* wifiSsid = "yourSSID";				// Typically kept in "privateInfo.h".
//const char* wifiPassword = "yourPassword";		// Typically kept in "privateInfo.h".
//const char* mqttBroker = "yourBrokerAddress";	// Typically kept in "privateInfo.h".
//const int mqttPort = 1883;							// Typically kept in "privateInfo.h".
const char* mqttTopic = "ajhWeather";
const String sketchName = "ESP32BME688";
const char* notes = "Adafruit ESP32-S2 QT Py BME688";
char ipAddress[16];
char macAddress[18];
int loopCount = 0;
int mqttPublishDelayMS = 60000;
int pirPin = 8;											// The GPIO that the PIR "out" pin is connected to.

// Create class objects.
WiFiClient espClient;							// Network client.
PubSubClient mqttClient( espClient );		// MQTT client.
Adafruit_NeoPixel pixels( NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800 );
Adafruit_BME680 bme;								// I2C.
//Adafruit_BME680 bme( BME_CS );				// Hardware SPI.
//Adafruit_BME680 bme( BME_CS, BME_MOSI, BME_MISO,  BME_SCK );		// Software SPI.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	while( !Serial )
		delay( 100 );
	Wire.setPins( SDA1, SCL1 );	// This is what selects the Stemma QT port, otherwise the two pin headers will be I2C.
	Wire.begin();

#if defined( NEOPIXEL_POWER )
	// If this board has a power control pin, we must set it to output and high in order to enable the NeoPixels.
	// We put this in an #ifdef so it can be reused for other boards without compilation errors.
	pinMode( NEOPIXEL_POWER, OUTPUT );
	digitalWrite( NEOPIXEL_POWER, HIGH );
#endif
	// Initialize the NeoPixel.
	pixels.begin();
	pixels.setBrightness( 20 );

	delay( 10 );
	Serial.println( '\n' );
	Serial.println( sketchName + " is beginning its setup()." );
//	Wire.begin();	// Join I2C bus.

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	Wire1.setPins( SDA1, SCL1 );

	Serial.println( "Initializing the BME2688 sensor..." );
	if( !bme.begin() )
	{
		Serial.println( "Could not find a valid BME2688 sensor, check wiring!" );
		while( 1 )
		{
			// Set color to red and wait a half second.
			pixels.fill( 0xFF0000 );
			pixels.show();
			delay( 500 );
			// Turn the LED off and wait a half second.
			pixels.fill( 0x000000 );
			pixels.show();
			delay( 500 );
		}
	}
	Serial.println( "Initializing the BME2688 sensor..." );

	// Set up oversampling and filter initialization
	bme.setTemperatureOversampling( BME680_OS_8X );
	bme.setHumidityOversampling( BME680_OS_2X );
	bme.setPressureOversampling( BME680_OS_4X );
	bme.setIIRFilterSize( BME680_FILTER_SIZE_3 );
	bme.setGasHeater( 320, 150 );		// 320*C for 150 ms

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );

	// Try to connect to the configured WiFi network, up to 10 times.
	wifiConnect( 20 );
} // End of setup() function.


void wifiConnect( int maxAttempts )
{
	// Announce WiFi parameters.
	String logString = "WiFi connecting to SSID: ";
	logString += wifiSsid;
	Serial.println( logString );

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : " - Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );

	int i = 0;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when WiFi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until WiFi has connected.
	while( WiFi.status() != WL_CONNECTED && i < maxAttempts )
	{
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		logString = ++i;
		logString += " seconds";
		Serial.println( logString );
	}

	// Print that WiFi has connected.
	Serial.println( '\n' );
	Serial.println( "WiFi connection established!" );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );
	Serial.print( "IP address: " );
	snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.println( ipAddress );
} // End of wifiConnect() function.


// mqttConnect() will attempt to (re)connect the MQTT client.
void mqttConnect( int maxAttempts )
{
	int i = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && i < maxAttempts )
	{
		Serial.print( "Attempting MQTT connection..." );
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		if( mqttClient.connect( macAddress ) )
		{
			Serial.println( "connected!" );
		}
		else
		{
			Serial.print( " failed, return code: " );
			Serial.print( mqttClient.state() );
			Serial.println( " try again in 2 seconds" );
			// Wait 5 seconds before retrying.
			delay( 5000 );
		}
		i++;
	}
} // End of mqttConnect() function.


void readBme()
{
	if( !bme.performReading() )
	{
		Serial.println( "Failed to perform reading :(" );
		return;
	}
	Serial.print( "Temperature = " );
	Serial.print( bme.temperature );
	Serial.println( " *C" );

	Serial.print( "Pressure = " );
	Serial.print( bme.pressure / 100.0 );
	Serial.println( " hPa" );

	Serial.print( "Humidity = " );
	Serial.print( bme.humidity );
	Serial.println( " %" );

	Serial.print( "Gas = " );
	Serial.print( bme.gas_resistance / 1000.0 );
	Serial.println( " KOhms" );

	Serial.print( "Approx. Altitude = " );
	Serial.print( bme.readAltitude( SEALEVELPRESSURE_HPA ) );
	Serial.println( " m" );

	Serial.println();
//	delay( 6000 );
}


/**
 * The loop() function begins after setup(), and repeats as long as the unit is powered.
 */
void loop()
{
	loopCount++;
	Serial.println();
	Serial.println( sketchName );

	// Set color to red and wait a half second.
	pixels.fill( 0xFF0000 );
	pixels.show();
	delay( 500 );


	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
	{
		// Reconnect to the MQTT broker.
		mqttConnect( 10 );
	}
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	// Read and print the BME688 data.
	readBme();

	// Set color to blue and wait a half second.
	pixels.fill( 0x0000FF );
	pixels.show();
	delay( 500 );

	// Prepare a String to hold the JSON.
	char mqttString[256];
	// Write the readings to the String in JSON format.
	snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.2f,\n\t\"humidity\": %.2f,\n\t\"pressure\": %.2f,\n\t\"altitude\": %.2f,\n\t\"gas\": %.2f,\n\t\"uptime\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, bme.temperature, bme.humidity, ( bme.pressure / 100.0 ), ( bme.readAltitude( SEALEVELPRESSURE_HPA ) ), ( bme.gas_resistance / 1000.0 ), loopCount, notes );
//	snprintf( mqttString, 256, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"uptime\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, loopCount, notes );
	if( mqttClient.connected() )
	{
		// Publish the JSON to the MQTT broker.
		mqttClient.publish( mqttTopic, mqttString );
	}
	else
	{
		Serial.println( "Lost connection to the MQTT broker between the start of this loop and now!" );
	}
	// Print the JSON to the Serial port.
	Serial.println( mqttString );

	String logString = "loopCount: ";
	logString += loopCount;
	Serial.println( logString );
	Serial.println( "Pausing for 60 seconds..." );

	// Set color to green and wait one minute.
	pixels.fill( 0x00FF00 );
	pixels.show();

	delay( 60000 );	// Wait for 60 seconds.
} // End of loop() function.
