//Spencer Butler, CS 452, Stepper Project
#ifndef stepperProject_h
#define stepperProject_h

#include <FreeRTOS.h>
#include <semphr.h>
#include "WS2812.hpp"


#define I2C_PORT i2c1
#define HDC1080ADDRESS 0x40

//time in ms within which button-changes are interpreted as bounces
#define BOUNCE_TIME (50)

//minimum time to display each value for on the seven-segment, in seconds
#define DISPLAY_TIME (5)

//time between measurements of temp/humidity, in seconds
#define SENSOR_INTERVAL (10)


//gpio pins for the pushbuttons
#define BUTTON_1 (19)
#define BUTTON_2 (9)
#define BUTTON_3 (8)

//gpio pin for the leftmost toggle switch
#define SWITCH_1 (6)

//neopixel control
#define LED_PIN (20)
#define LED_LENGTH (4)
//delay in ms of each step of the rainbow animation
//also delayed by each step polling the switch to support ending the animation early
#define RAINBOW_STEPSIZE (40)

//which led for which tasks
#define DISPLAY_TYPE_LED (0)
#define DISPLAY_MODE_LED (1)
#define MOTOR_GOAL_LED (2)
#define MOTOR_STABLE_LED (3)
//how many steps can the motor take before the value is considered unstable
#define MAX_STABLE_STEPS (36)

//gpio pins for the stepper motor
#define STEPPER_1 (0)
#define STEPPER_2 (1)
#define STEPPER_3 (12)
#define STEPPER_4 (13)

//total steps in a circle
//28byj-48 datasheet listed 5.625/64 degrees per step
//that would give 4096 steps per circle
//4096 steps gave exactly 2 full revolutions -- these steps are twice as big
#define TOTAL_STEPS (2048)

//common-cathodes for seven segment display
#define RIGHT_CC (10)
#define LEFT_CC (11)


//statevar values
#define SS_TEMPERATURE (1)
#define SS_HUMIDITY (2)
#define SS_ROTATION (3)
#define SS_SPECIAL (4)

#define SS_HEX (true)
#define SS_DEC (false)

#define SM_TEMPERATURE (1)
#define SM_HUMIDITY (2)
#define SM_ALTERNATING (3)
#define SM_SPECIAL (4)

extern double humidity;
extern double temperature;


extern uint8_t displayValue;

SemaphoreHandle_t sensorBusy;
//initializes the hdc1080 sensor
void initializeSensor();

//returns unparsed temperature and humidity data
//ret[0] is temperature, r[1] is humidity
void readSensor(uint16_t ret[2]);

//parses raw temperature data into degrees celsius
double parseTemperatureC(uint16_t raw);

//converts degrees celsius to fahrenheit
double degCtoF(double degC);

//parses raw humidity data into a decimal from 0 to 1 
double parseHumidity(uint16_t raw);

//busy delay to support multiple motor steps per tick
uint16_t busyDelay(uint16_t t);

//calculate goalPosition based on global humidity
int hToGoal();

//calculate goalPosition based on global temperature
int tToGoal();

//colors for pixel output
#define COLOR_OFF (WS2812::RGB(0, 0, 0))
#define COLOR_RED (WS2812::RGB(255, 0, 0))
#define COLOR_GREEN (WS2812::RGB(0, 255, 0))
#define COLOR_BLUE (WS2812::RGB(0, 0, 255))
#define COLOR_YELLOW (WS2812::RGB(255, 255, 0))
#define COLOR_WHITE (WS2812::RGB(255, 255, 255))

#define COLOR_STABLE (COLOR_GREEN)
#define COLOR_UNSTABLE (COLOR_RED)

#define COLOR_TEMPERATURE (COLOR_YELLOW)
#define COLOR_HUMIDITY (COLOR_BLUE)
#define COLOR_ROTATION (COLOR_GREEN)
#define COLOR_ALTERNATING (COLOR_GREEN)
#define COLOR_SPECIAL (COLOR_WHITE)

#define COLOR_HEX (COLOR_YELLOW)
#define COLOR_DEC (COLOR_BLUE)

//global state variables
extern int displayType;
extern int displayMode;
extern int motorGoal;


extern SemaphoreHandle_t positionRequested;
extern QueueHandle_t positionGoals;

extern QueueHandle_t buttonPresses;

extern QueueHandle_t valuesToDisplay;
//struct for sending messages to the display controller
typedef struct displayFrame {
   int valueType;
   double value;
} displayFrame;

extern QueueHandle_t pixelChanges;
//struct for sending messages to the pixel controller
typedef struct pixelFrame {
   int pixel;
   uint32_t color;
} pixelFrame;


#endif





