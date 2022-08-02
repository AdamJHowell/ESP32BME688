/*
 * This sketch is a branch of my PubSubWeather sketch.
 * This sketch will use a BME688 sensor to show temperature, pressure, humidity, and gas readings.
 * @copyright   Copyright © 2022 Adam Howell
 * @licence     The MIT License (MIT)
 */
#include <WiFi.h>						// This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <Adafruit_Sensor.h>		// Adafruit Unified Sensor Driver.  https://github.com/adafruit/Adafruit_Sensor
#include "Adafruit_BME680.h"		// Adafruit BME680 (and BME688) library.  https://github.com/adafruit/Adafruit_BME680
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.
#include <Adafruit_NeoPixel.h>	// The Adafruit NeoPixel library to drive the RGB LED on the QT Py.	https://github.com/adafruit/Adafruit_NeoPixel


// NUMPIXELS sets the number of NeoPixel elements.
#define NUMPIXELS        1
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define RED 0xFF0000
#define ORANGE 0xFFA500
#define YELLOW 0xFFFF00
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define INDIGO 0x4B0082
#define VIOLET 0xEE82EE
#define PURPLE 0x800080
#define BLACK 0x000000
#define GRAY 0x808080
#define WHITE 0xFFFFFF


/**
 * Declare global variables.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
//const char* wifiSsid = "yourSSID";				// Typically kept in "privateInfo.h".
//const char* wifiPassword = "yourPassword";		// Typically kept in "privateInfo.h".
//const char* mqttBroker = "yourBrokerAddress";	// Typically kept in "privateInfo.h".
//const int mqttPort = 1883;							// Typically kept in "privateInfo.h".
const char* mqttTopic = "espWeather";
const String sketchName = "QTPyBME688";
const char* notes = "Adafruit ESP32-S2 QT Py BME688";
char ipAddress[16];
char macAddress[18];
int loopCount = 0;
int publishDelay = 60000;
float SEALEVELPRESSURE_HPA = 1025.0;
unsigned long lastPublish = 0;


// Create class objects.
WiFiClient espClient;							// Network client.
PubSubClient mqttClient( espClient );		// MQTT client.
Adafruit_NeoPixel pixels( NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800 );
Adafruit_BME680 bme;								// I2C.
//Adafruit_BME680 bme( BME_CS );				// Hardware SPI.
//Adafruit_BME680 bme( BME_CS, BME_MOSI, BME_MISO,  BME_SCK );		// Software SPI.


void setup()
{
	delay( 500 );

#if defined( NEOPIXEL_POWER )
	// If this board has a power control pin, we must set it to output and high in order to enable the NeoPixels.
	// We put this in an #ifdef so it can be reused for other boards without compilation errors.
	pinMode( NEOPIXEL_POWER, OUTPUT );
	digitalWrite( NEOPIXEL_POWER, HIGH );
#endif
	// Initialize the NeoPixel.
	pixels.begin();
	pixels.setBrightness( 20 );

	// Set the LED color to gray to indicate setup is underway.
	pixels.fill( GRAY );
	pixels.show();

	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );
	Wire.setPins( SDA1, SCL1 );	// This is what selects the Stemma QT port, otherwise the two pin headers will be I2C.
	Wire.begin();

	Serial.println();
	Serial.println( sketchName + " is beginning its setup()." );
	Serial.println( __FILE__ );

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );

	// Try to connect to the configured WiFi network, up to 20 times.
	wifiConnect( 20 );

	Serial.println( "Initializing the BME688 sensor..." );
	if( !bme.begin() )
	{
		while( 1 )
		{
			Serial.println( "Could not find a valid BME688 sensor, check wiring!" );
			// Set the LED color to red and wait one second.
			pixels.fill( RED );
			pixels.show();
			delay( 1000 );
			// Set the LED color to yellow and wait a half second.
			pixels.fill( YELLOW );
			pixels.show();
			delay( 500 );
		}
	}
	Serial.println( "BME688 has been initialized." );

	// Set up oversampling and filter initialization
	bme.setTemperatureOversampling( BME680_OS_8X );
	bme.setHumidityOversampling( BME680_OS_2X );
	bme.setPressureOversampling( BME680_OS_4X );
	bme.setIIRFilterSize( BME680_FILTER_SIZE_3 );
	bme.setGasHeater( 320, 150 );		// 320*C for 150 ms
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
		Serial.println( "Waiting for a connection..." );
		// Set the LED color to red and wait one second.
		pixels.fill( RED );
		pixels.show();
		delay( 1000 );
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

	// Set the LED color to green.
	pixels.fill( GREEN );
	pixels.show();
} // End of wifiConnect() function.


// mqttConnect() will attempt to (re)connect the MQTT client.
void mqttConnect( int maxAttempts )
{
	int i = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && i < maxAttempts )
	{
		Serial.print( "Attempting MQTT connection..." );
		// Set the LED color to yellow.
		pixels.fill( ORANGE );
		pixels.show();
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		if( mqttClient.connect( macAddress ) )
		{
			Serial.println( "connected!" );
			// Set the LED color to green.
			pixels.fill( GREEN );
			pixels.show();
		}
		else
		{
			// Set the LED color to red.
			pixels.fill( RED );
			pixels.show();
			Serial.print( " failed, return code: " );
			Serial.print( mqttClient.state() );
			Serial.println( " try again in 2 seconds" );
			// Wait 5 seconds before retrying.
			delay( 5000 );
		}
		i++;
	}
	mqttClient.setBufferSize( 512 );
	char mqttString[512];
	snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\"\n}", sketchName, macAddress, ipAddress );
	mqttClient.publish( "espConnect", mqttString );
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
}


void loop()
{
	// Set the LED color to orange.
	pixels.fill( ORANGE );
	pixels.show();
	delay( 10 );

	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
	{
		// Reconnect to the MQTT broker.
		mqttConnect( 10 );
	}
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	unsigned long time = millis();

	if( ( time > publishDelay ) && ( time - publishDelay ) > lastPublish )
	{
		loopCount++;
		Serial.println();
		Serial.println( sketchName );
		Serial.print( "Connected to broker at \"" );
		Serial.print( mqttBroker );
		Serial.print( ":" );
		Serial.print( mqttPort );
		Serial.println( "\"" );

		// Print the signal strength:
		long rssi = WiFi.RSSI();
		Serial.print( "WiFi RSSI: " );
		Serial.println( rssi );

		// Read and print the BME688 data.
		readBme();

		// Set the LED color to blue and wait a half second.
		pixels.fill( BLUE );
		pixels.show();
		delay( 500 );

		// Prepare a String to hold the JSON.
		char mqttString[512];
		// Write the readings to the String in JSON format.
		snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.2f,\n\t\"humidity\": %.2f,\n\t\"pressure\": %.2f,\n\t\"altitude\": %.2f,\n\t\"gas\": %.2f,\n\t\"rssi\": %ld,\n\t\"uptime\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, bme.temperature, bme.humidity, ( bme.pressure / 100.0 ), ( bme.readAltitude( SEALEVELPRESSURE_HPA ) ), ( bme.gas_resistance / 1000.0 ), rssi, loopCount, notes );
		if( mqttClient.connected() )
		{
			Serial.print( "Publishing to topic \"" );
			Serial.print( mqttTopic );
			Serial.println( "\"" );
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

		lastPublish = millis();
		Serial.print( "Next publish in " );
		Serial.print( publishDelay / 1000 );
		Serial.println( " seconds.\n" );
	}
	// Set the LED color to green.
	pixels.fill( GREEN );
	pixels.show();
	delay( 100 );
} // End of loop() function.
