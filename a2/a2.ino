/*
* CSCI 1063U - Elegoo Smart Car V4.0
*
* Starter Code for motor, pin, and sensor setup
* Provided to students for use and understanding
* 
*/

#include "config.hpp"
#include "pins.hpp"
#include "utils.hpp"

#include "roaming.hpp"
#include "following.hpp"
#include "searching.hpp"
#include "line_following.hpp"
#include "sticky_following.hpp"

SensorData sensor_data;
extern int state;

void setup() {
	setupArduino();
	
	// Wait for button press
	while (digitalRead(BUTTON) == HIGH) {
		Serial.println("Button not pressed");
	}
	
	delay(500);
	
	// Initialize Gyro - hard stop if failed
	if (!setupGyro()) {
		Serial.println("Failed setting up gyro...");
		ledOn(CRGB::Red);
		while (true);  // Hard stop
	}
	
	calibrateGyro();
	
	// Custom
	randomSeed(analogRead(0));
	
	print("MAIN", "setup done", 0);
}

// STATES
// -1 TESTING PURPOSES (I'm sick and tired of the motor noises)
// 0 ROAMING
// 1 FOLLOWING
// 2 SEARCHING
// 3 FOLLOW LINE

void loop()
{
	sensor_data = update_sensors();
	state = -1;

	// print("main", "state", state);
	// switch (-1)
	switch (state)
	{
	case -1:
		sticky_following();
		break;
	
	case 0:
		roaming();
		break;
	
	case 1:
		following();
		break;
	
	case 2:
		searching();
		break;
	
	case 3:
		follow_line();
		break;
	
	default:
		break;
	}
}
