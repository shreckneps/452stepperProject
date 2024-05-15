//Spencer Butler, CS 452, Stepper Project
#include <stdio.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <time.h>
#include <math.h>


#include "WS2812.hpp"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "stepperProject.h"

//gpio 7, 18, and 24-29 correspond to 7-segment input values
//32 bit is 8 hex
//7     -- hex 1, bit 3
//18    -- hex 4, bit 2
//24-27 -- hex 6, all bits
//28-29 -- hex 7, bits 0-1
const uint32_t bitMask = 0x3f040080;

//font for displaying digits 0-9
//0x3e040080 is all segments on, dp off
const uint32_t font[16] = {
  0x2e040080, // 0
  0x28000000, // 1
  0x1e040000, // 2
  0x3c040000, // 3
  0x38000080, // 4
  0x34040080, // 5
  0x36040080, // 6
  0x2c000000, // 7
  0x3e040080, // 8
  0x3c040080, // 9
  0x3e000080, // A
  //0x3e040000, // a
  0x32040080, // b
  0x12040000, // c
  0x3a040000, // d
  0x1e040080, // e
  0x16000080  // F
};
uint8_t displayValue;

int displayType = SS_TEMPERATURE;
int displayMode = SS_DEC;
int motorGoal = SM_TEMPERATURE;

SemaphoreHandle_t positionRequested;
QueueHandle_t positionGoals;
QueueHandle_t buttonPresses;
QueueHandle_t valuesToDisplay;
QueueHandle_t pixelChanges;

//#define TASK_LIST_BUFFER_SIZE (640)


//task for reporting temperature/humidity every 10 seconds
void task_valueMonitor(void *p) {
   uint16_t rawValues[2];
   displayFrame df;
   int buffer;

   while(1) {
      readSensor(rawValues);
      //fahrenheit more informative when displaying in 0-99 or 0-255 degree range
      temperature = degCtoF(parseTemperatureC(rawValues[0]));
      humidity = parseHumidity(rawValues[1]);

      if(motorGoal == SM_TEMPERATURE) {
         buffer = tToGoal();
         xQueueSendToBack(positionGoals, &buffer, 0);
      } else if(motorGoal == SM_HUMIDITY) {
         buffer = hToGoal();
         xQueueSendToBack(positionGoals, &buffer, 0);
      }

      if(displayType == SS_TEMPERATURE) {
         df.valueType = SS_TEMPERATURE;
         df.value = temperature;
         xQueueSendToBack(valuesToDisplay, &df, 5);
      } else if(displayType == SS_HUMIDITY) {
         df.valueType = SS_HUMIDITY;
         df.value = humidity;
         xQueueSendToBack(valuesToDisplay, &df, 5);
      }

      vTaskDelay((SENSOR_INTERVAL * 1000) / portTICK_PERIOD_MS);
   }
}

//task that moves the motor cw or ccw to the desired position
void task_stepperDriver(void *p) {
   int pins[4] = {
      STEPPER_1,
      STEPPER_2,
      STEPPER_3,
      STEPPER_4
   };

   int on = 0;
   //define the initial position to be the center of the dial
   int currentPosition = TOTAL_STEPS / 2;
   int goalPosition = currentPosition;
   int i, nextOn, direction;
   int stepsTaken = 0;

   displayFrame df;
   pixelFrame pf;
   pf.pixel = MOTOR_STABLE_LED;


   gpio_put(pins[0], 1);
   for(i = 1; i < 4; i++) {
      gpio_put(pins[i], 0);
   }
   

   while(1) {
      //if the position is requested for display on the screen then send it and clear request
      if(uxSemaphoreGetCount(positionRequested)) {
         df.valueType = SS_ROTATION;
         df.value = ((double) currentPosition) / TOTAL_STEPS;
         //printf("Position requested, delivered: %lf\n", df.value);
         xQueueSendToBack(valuesToDisplay, &df, 0);
         xSemaphoreTake(positionRequested, 1);
      }
      while(xQueueReceive(positionGoals, &i, 0)) {
         goalPosition = i;
         //printf("Position goal set to: %d\n", goalPosition);
      }
      //-1 used as special instruction to stop the motor
      if(goalPosition == -1) {
         goalPosition = currentPosition;
      }

      if(goalPosition > currentPosition) {
         //positive means clockwise
         direction = 1;
      } else if(goalPosition < currentPosition) {
         direction = -1;
      } else {
         direction = 0;
         //if moved recently, to avoid spamming the queue
         if(stepsTaken) {
            stepsTaken = 0;
            pf.color = COLOR_STABLE;
            xQueueSendToBack(pixelChanges, &pf, 0);
         }
      }
      
      if(direction) {
         nextOn = on + direction;
         if(nextOn > 3) {
            nextOn = 0;
         } else if(nextOn < 0) {
            nextOn = 3;
         }

         currentPosition += direction;
         if(currentPosition < 1) {
            if(motorGoal == SM_ALTERNATING) {
               //printf("Went all the way left, flipping.\n");
               goalPosition *= -1;
            } else {
               currentPosition = TOTAL_STEPS;
            }
         } else if(currentPosition > TOTAL_STEPS) {
            if(motorGoal == SM_ALTERNATING) {
               //printf("Went all the way right, flipping.\n");
               goalPosition *= -1;
            } else {
               currentPosition = 1;
            }
         }

         stepsTaken++;
         //only upon reaching the threshold, to avoid spamming the queue
         if(stepsTaken == MAX_STABLE_STEPS) {
            pf.color = COLOR_UNSTABLE;
            xQueueSendToBack(pixelChanges, &pf, 0);
         }

         nextOn = nextOn % 4;
         gpio_put(pins[nextOn], 1);
         //busy wait rather than vTaskDelay to increase speed and fluidity of motion
         //other tasks preempt this one, so not yielding doesn't cause a problem
         //minimum pulse width for stepper to work seems to be more than 4096
         busyDelay(8192);
         gpio_put(pins[on], 0);
         on = nextOn;
         busyDelay(8192);
      } else {
         vTaskDelay(1);
      }
   }

}

//minimal task to keep displayValue shown on the seven-segment display
void task_displayDriver(void *p) {
   uint8_t localValue;
   while(1) {
      localValue = displayValue;
      
      gpio_put(LEFT_CC, 1);
      gpio_put(RIGHT_CC, 0);
      gpio_put_masked(bitMask, font[localValue & 0x0f]);
      vTaskDelay(1);

      gpio_put(LEFT_CC, 0);
      gpio_put(RIGHT_CC, 1);
      gpio_put_masked(bitMask, font[localValue >> 4]);
      vTaskDelay(1);
   }
}

//task to periodically update which value is displayed
void task_displayControl(void *p) {
   displayFrame df;
   pixelFrame pf;
   int tempVal;
   while(1) {
      if(displayType == SS_ROTATION) {
         //the motor rotates much faster than any reasonable show-same-value-for-X-seconds time
         //so motor drive task only sends position when requested
         xSemaphoreGive(positionRequested);
      }

      if(xQueueReceive(valuesToDisplay, &df, 5)) { 
         if(uxQueueMessagesWaiting(valuesToDisplay) < 10) {
            //printf("Displaying: (%d, %lf)\n", df.valueType, df.value);
            pf.pixel = DISPLAY_TYPE_LED;
            switch(df.valueType) {
               case SS_TEMPERATURE:
                  pf.color = COLOR_TEMPERATURE;
                  tempVal = (int) df.value;
                  if(tempVal < 0) { tempVal = 0; }
                  if(displayMode == SS_DEC) {
                     if(tempVal > 100) { tempVal = 100; }
                     tempVal = ((tempVal / 10) << 4) + (tempVal % 10);
                  }
                  displayValue = tempVal;

                  break;
               case SS_HUMIDITY:
                  pf.color = COLOR_HUMIDITY;
                  if(displayMode == SS_HEX) {
                     tempVal = (int) (df.value * 256);
                  } else {
                     tempVal = (int) (df.value * 100);
                     tempVal = ((tempVal / 10) << 4) + (tempVal % 10);
                  }
                  displayValue = tempVal;
                  break;
               case SS_ROTATION:
                  pf.color = COLOR_ROTATION;
                  if(displayMode == SS_HEX) {
                     tempVal = (int) (df.value * 256);
                  } else {
                     tempVal = (int) (df.value * 100);
                     tempVal = ((tempVal / 10) << 4) + (tempVal % 10);
                  }
                  displayValue = tempVal;
                  break;
               default:
                  //printf("Received unexpected value type in display queue: (%d, %lf)\n", df.valueType, df.value);
                  break;
            }
            xQueueSendToBack(pixelChanges, &pf, 0);
            pf.pixel = DISPLAY_MODE_LED;
            if(displayMode == SS_HEX) {
               pf.color = COLOR_HEX;
            } else {
               pf.color = COLOR_DEC;
            }
            xQueueSendToBack(pixelChanges, &pf, 0);

            vTaskDelay((DISPLAY_TIME * 1000) / portTICK_PERIOD_MS);
   
            
         } else {
            xQueueReset(valuesToDisplay);
            displayValue = 0x0f;

            pf.pixel = DISPLAY_TYPE_LED;
            pf.color = COLOR_SPECIAL;
            xQueueSendToBack(pixelChanges, &pf, 0);

            pf.pixel = DISPLAY_MODE_LED;
            pf.color = COLOR_SPECIAL;
            xQueueSendToBack(pixelChanges, &pf, 0);

            vTaskDelay((5 * 1000) / portTICK_PERIOD_MS);
         }
      }
   }
}

//ISR handling button presses; does simple time-since-last-change debouncing
void buttonISR(unsigned int pin, uint32_t event) {
   static uint32_t last1 = 0;
   static uint32_t last2 = 0;
   static uint32_t last3 = 0;
   static int buttonValues[3] = {0, 1, 2};

   //printf("ISR\n");

   switch(pin) {
      case BUTTON_1:
         if(event & GPIO_IRQ_EDGE_RISE) {
            //not sure how heavy these time functions are -- may cause problems in an ISR
            if((to_ms_since_boot(get_absolute_time()) - last1) > BOUNCE_TIME) {
               xQueueSendToBackFromISR(buttonPresses, buttonValues, NULL);
            }
         }
         last1 = to_ms_since_boot(get_absolute_time());
         break;

      case BUTTON_2:
         if(event & GPIO_IRQ_EDGE_RISE) {
            if((to_ms_since_boot(get_absolute_time()) - last2) > BOUNCE_TIME) {
               xQueueSendToBackFromISR(buttonPresses, buttonValues + 1, NULL);
            }
         }
         last2 = to_ms_since_boot(get_absolute_time());
         break;

      case BUTTON_3:
         if(event & GPIO_IRQ_EDGE_RISE) {
            if((to_ms_since_boot(get_absolute_time()) - last3) > BOUNCE_TIME) {
               xQueueSendToBackFromISR(buttonPresses, buttonValues + 2, NULL);
            }
         }
         last3 = to_ms_since_boot(get_absolute_time());
         break;

      default:
         break;
   }
}


//task to respond to button-presses sent by the interrupt handler
void task_buttonResponse(void *p) {
   uint32_t firstPressed[3] = {0, 0, 0};
   uint32_t now;

   int numPresses[3] = {0, 0, 0};
   int button, buffer;
   
   //to enable reaching overflow state, send a display-this-value message
   //every time a change-display-type control is entered
   displayFrame df;
   pixelFrame pf;
   pf.pixel = MOTOR_GOAL_LED;

   while(1) {
      //timeout every 5 ticks to process the 2-second control frames
      if(xQueueReceive(buttonPresses, &button, 5)) {
         now = to_ms_since_boot(get_absolute_time());
         if(numPresses[button]) {
            numPresses[button] = numPresses[button] + 1;
         } else {
            numPresses[button] = 1;
            firstPressed[button] = now;
         }
         //printf("Button pressed: %d\n", button);

      } else {
         now = to_ms_since_boot(get_absolute_time());

         if(numPresses[0] && (now - firstPressed[0] > 2000)) {
            //printf("Button 1: %d presses\n", numPresses[0]);
            switch(numPresses[0]) {
               case 1:
                  //move stepper based on humidity
                  motorGoal = SM_HUMIDITY;
                  buffer = hToGoal();
                  xQueueSendToBack(positionGoals, &buffer, 0);
                  pf.color = COLOR_HUMIDITY;
                  xQueueSendToBack(pixelChanges, &pf, 0);
                  break;
               case 2:
                  //move on temperature
                  motorGoal = SM_TEMPERATURE;
                  buffer = tToGoal();
                  pf.color = COLOR_TEMPERATURE;
                  xQueueSendToBack(pixelChanges, &pf, 0);
                  xQueueSendToBack(positionGoals, &buffer, 0);
                  break;
               case 3:
                  //stop everything, display EE
                  displayType = SS_SPECIAL;
                  motorGoal = SM_SPECIAL;
                  //clear and bypass the normal control queue for immediate display
                  xQueueReset(valuesToDisplay);
                  displayValue = 0xee;
                  buffer = -1;
                  xQueueSendToBack(positionGoals, &buffer, 0);
                  pf.pixel = DISPLAY_TYPE_LED;
                  pf.color = COLOR_SPECIAL;
                  xQueueSendToBack(pixelChanges, &pf, 0);
                  pf.pixel = DISPLAY_MODE_LED;
                  pf.color = COLOR_SPECIAL;
                  xQueueSendToBack(pixelChanges, &pf, 0);

                  pf.pixel = MOTOR_GOAL_LED;
                  pf.color = COLOR_SPECIAL;
                  xQueueSendToBack(pixelChanges, &pf, 0);
                  break;
               case 4:
               default:
                  //toggle display dec/hex
                  displayMode = !displayMode;
                  break;
            }
            numPresses[0] = 0;
         }

         if(numPresses[1] && (now - firstPressed[1] > 2000)) {
            //printf("Button 2: %d presses\n", numPresses[1]);
            switch(numPresses[1]) {
               case 1:
                  //move stepper clockwise continually
                  motorGoal = SM_SPECIAL;
                  buffer = 2 * TOTAL_STEPS;
                  xQueueSendToBack(positionGoals, &buffer, 0);
                  pf.color = COLOR_SPECIAL;
                  xQueueSendToBack(pixelChanges, &pf, 0);
                  break;
               case 2:
                  //move ccw
                  motorGoal = SM_SPECIAL;
                  buffer = -2 * TOTAL_STEPS;
                  xQueueSendToBack(positionGoals, &buffer, 0);
                  pf.color = COLOR_SPECIAL;
                  xQueueSendToBack(pixelChanges, &pf, 0);
                  break;
               case 3:
               default:
                  //alternate revolutions cw, ccw
                  motorGoal = SM_ALTERNATING;
                  buffer = 2 * TOTAL_STEPS;
                  xQueueSendToBack(positionGoals, &buffer, 0);
                  pf.color = COLOR_ALTERNATING;
                  xQueueSendToBack(pixelChanges, &pf, 0);
                  break;
            }
            numPresses[1] = 0;
         }

         if(numPresses[2] && (now - firstPressed[2] > 2000)) {
            //printf("Button 3: %d presses\n", numPresses[2]);
            switch(numPresses[2]) {
               case 1:
                  //display temperature
                  displayType = SS_TEMPERATURE;
                  df.valueType = SS_TEMPERATURE;
                  df.value = temperature;
                  xQueueSendToBack(valuesToDisplay, &df, 5);
                  break;
               case 2:
                  //display humidity
                  displayType = SS_HUMIDITY;
                  df.valueType = SS_HUMIDITY;
                  df.value = humidity;
                  xQueueSendToBack(valuesToDisplay, &df, 5);
                  break;
               case 3:
               default:
                  //display stepper motor rotation status
                  displayType = SS_ROTATION;
                  xSemaphoreGive(positionRequested);
                  break;
            }
            numPresses[2] = 0;
         }
      }
   }
}

void task_pixelControl(void *p) {
   pixelFrame pf;
   //WGRB is not one of their default formats.
   //Adding it took 3ish lines in the .hpp and .cpp
   //This is the format that made the example act as described
   WS2812 strip(LED_PIN, LED_LENGTH, pio0, 0, WS2812::FORMAT_WGRB);
   uint32_t localColors[LED_LENGTH];
   uint32_t randomColor;
   int i;
   int delta, wasRainbow;

   for(i = 0; i < LED_LENGTH; i++) {
      localColors[i] = COLOR_OFF;
   }
   localColors[MOTOR_GOAL_LED] = COLOR_TEMPERATURE;

   while(1) {
      delta = false;
      while(xQueueReceive(pixelChanges, &pf, 5)) {
         //printf("Received pixel frame: (%d, %lx)\n", pf.pixel, pf.color);
         if(localColors[pf.pixel] != pf.color) {
            localColors[pf.pixel] = pf.color;
            delta = true;
         }
      }

      //if the switch is flipped, do the rainbow rather than normal display
      if(!gpio_get(SWITCH_1)) {
         wasRainbow = true;
         //printf("Should rainbow now.\n");
         randomColor = (uint32_t)rand();
         if(rand() & 1) {
            for(i = 0; i < LED_LENGTH && !gpio_get(SWITCH_1); i++) {
               strip.setPixelColor(i, randomColor);
               strip.show();
               vTaskDelay(RAINBOW_STEPSIZE / portTICK_PERIOD_MS);
            }
         } else {
            for(i = LED_LENGTH - 1; i >= 0 && !gpio_get(SWITCH_1); i--) {
               strip.setPixelColor(i, randomColor);
               strip.show();
               vTaskDelay(RAINBOW_STEPSIZE / portTICK_PERIOD_MS);
            }
         }
         
      } else if(wasRainbow || delta) {
         //update the strip if something changed, or the rainbow ended
         for(i = 0; i < LED_LENGTH; i++) {
            strip.setPixelColor(i, localColors[i]);
         }
         strip.show();
      }


      
   }
}

#ifdef TASK_LIST_BUFFER_SIZE
void task_debugMonitor(void *p) {
   char buffer[TASK_LIST_BUFFER_SIZE];
   while(1) {
      vTaskList(buffer);
      printf("%s\n\n\n", buffer);
      vTaskDelay((10 * 1000) / portTICK_PERIOD_MS);
   }
}
#endif


int main() {
   //usb communication
   stdio_init_all();

   //hdc1080
   initializeSensor();
   
   //stepper motor
   gpio_init(STEPPER_1);
   gpio_set_dir(STEPPER_1, GPIO_OUT);
   gpio_init(STEPPER_2);
   gpio_set_dir(STEPPER_2, GPIO_OUT);
   gpio_init(STEPPER_3);
   gpio_set_dir(STEPPER_3, GPIO_OUT);
   gpio_init(STEPPER_4);
   gpio_set_dir(STEPPER_4, GPIO_OUT);
   positionGoals = xQueueCreate(16, sizeof(int));
   positionRequested = xSemaphoreCreateBinary();
   
   //leftmost toggle-switch
   gpio_init(SWITCH_1);
   gpio_set_dir(SWITCH_1, GPIO_IN);
   
   //seven-segment display
   gpio_init(RIGHT_CC);
   gpio_set_dir(RIGHT_CC, GPIO_OUT);
   gpio_init(LEFT_CC);
   gpio_set_dir(LEFT_CC, GPIO_OUT);
   gpio_init_mask(bitMask);
   gpio_set_dir_out_masked(bitMask);
   valuesToDisplay = xQueueCreate(16, sizeof(displayFrame));

   //buttons
   gpio_init(BUTTON_1);
   gpio_set_dir(BUTTON_1, GPIO_IN);
   gpio_init(BUTTON_2);
   gpio_set_dir(BUTTON_2, GPIO_IN);
   gpio_init(BUTTON_3);
   gpio_set_dir(BUTTON_3, GPIO_IN);
   buttonPresses = xQueueCreate(16, sizeof(int));
   irq_set_enabled(IO_IRQ_BANK0, true);
   gpio_set_irq_callback(&buttonISR);
   gpio_set_irq_enabled(BUTTON_1, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
   gpio_set_irq_enabled(BUTTON_2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
   gpio_set_irq_enabled(BUTTON_3, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

   pixelChanges = xQueueCreate(16, sizeof(pixelFrame));
   
   xTaskCreate(task_valueMonitor, "ValueMonitor", 256, NULL, 3, NULL);
   xTaskCreate(task_stepperDriver, "StepperDriver", 256, NULL, 1, NULL);
   xTaskCreate(task_displayDriver, "DisplayDriver", 256, NULL, 5, NULL);
   xTaskCreate(task_displayControl, "DisplayControl", 256, NULL, 3, NULL);
   xTaskCreate(task_buttonResponse, "ButtonResponse", 256, NULL, 3, NULL);
   xTaskCreate(task_pixelControl, "PixelControl", 1024, NULL, 4, NULL);
   #ifdef TASK_LIST_BUFFER_SIZE
      xTaskCreate(task_debugMonitor, "DebugMonitor", 1024, NULL, 4, NULL);
   #endif
   vTaskStartScheduler();


   while(1) { };
}

void initializeSensor() {
   uint8_t configLocation = 0x02;
   uint16_t configVal = 0x1000;

   //configure pins for i2c
   i2c_init(I2C_PORT, 100 * 1000);
   gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
   gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

   //configure the sensor
   i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &configLocation, 1, true);
   i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, (uint8_t *)&configVal, 2, false);
   
   //create the access-control semaphore
   sensorBusy = xSemaphoreCreateMutex();
   xSemaphoreGive(sensorBusy);
}

//ret[0] is temperature, r[1] is humidity
void readSensor(uint16_t ret[2]) {
   if(xSemaphoreTake(sensorBusy, portMAX_DELAY)) {
      uint8_t destination = 0x00;
      uint8_t helper[4];


      i2c_write_blocking(I2C_PORT, HDC1080ADDRESS, &destination, 1, false);
      
      //forced delay to allow time for measurement
      //datasheet specifies 6.5 ms as highest typical conversion time for temperature, 6.25 for humidity
      //I am unsure if these measurements are performed truly in parallel or in sequence
      //Saw strange measurements reported with 10ms, increased to 20 to be above 6.5 + 6.25
      //sleep_ms(20);
      vTaskDelay((20) / portTICK_PERIOD_MS);

      i2c_read_blocking(I2C_PORT, HDC1080ADDRESS, helper, 4, false);
      //printf("Raw called, helper bytes: 0x%hX 0x%hx 0x%hx 0x%hx\n", helper[0], helper[1], helper[2], helper[3]);
      ret[0] = helper[0]<<8 | helper[1];
      ret[1] = helper[2]<<8 | helper[3];

      xSemaphoreGive(sensorBusy);
   }
}


double parseTemperatureC(uint16_t raw) {
   //from datasheet, divide by 2^16, mult by 165, subtract 40
   double ret = ((double) raw) / (0xffff + 1.0);
   ret *= 165;
   ret -= 40;
   return ret;
}

double degCtoF(double degC) {
   return ((degC * 9.0 / 5.0) + 32);
}

double parseHumidity(uint16_t raw) {
   //from datasheet, divide by 2^16
   return (((double) raw) / (0xffff + 1.0));
}

uint16_t busyDelay(uint16_t t) {
   volatile uint16_t i, ret;
   for(i = 0; i < t; i++) {
      ret = t - i;
   }
   return ret;
}

//calculate goalPosition based on global humidity
int hToGoal() {
   //constrain dial arc to 270 --  each endpoint 1/8th away
   return (humidity * (TOTAL_STEPS * 0.75) + (TOTAL_STEPS * 0.125));
}

//calculate goalPosition based on global temperature
int tToGoal() {
   //take -100 to 200 as range of temperature values
   double distance = (temperature + 100) / 300.0;
   if(distance < 0) {
      distance = 0;
   } else if(distance > 1) {
      distance = 1;
   }
   return (distance * (TOTAL_STEPS * 0.75) + (TOTAL_STEPS * 0.125));
}

