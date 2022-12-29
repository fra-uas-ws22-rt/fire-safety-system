#include <Arduino_FreeRTOS.h>
#include <MQ2.h>

/**********Decalarations of Tasks************************/
void TaskBlink( void *pvParameters );
void TaskReadMQSensor( void *pvParameters );
void TaskRaiseAlarm( void *pvParameters );
void TaskReadKeypad( void *pvParameters );

/**********Decalarations of Functions********************/
void armSystem();
void disarmSystem();
void cancelAlarm();
void activateRedLED();
void activateGreenLED();
void activateYellowLED();

/**********Decalarations of Hardware Pins****************/
int smokePin = A0;
int buzzerPin = A1;

/**********Decalarations of System states****************/
enum SystemState
{
    INIT=0,
    ARMED=1,
    DISARMED=2,
    ALARM=3,
    FAULT=4
};

/**********Decalarations of Preprossors*******************/
#define   SMOKE_THRESHOLD_LEVEL   300
#define   BUZZER_TONE             3000
#define   SYSTEM_DISARM_CODE      "1234"
#define   SYSTEM_ARM_CODE         "4321"
#define   SYSTEM_CANCEL_ALARM     "0000"

/**********Decalarations of Global Varaibles**************/

enum SystemState state;
float smoke;
//MQ2 mq2(smokePin);

/**********Application Setup******************************/
void setup() {
  
  state = INIT;
  Serial.begin(9600); // DEBUGGING
  pinMode(smokePin, INPUT);     // Set smoke pin as input 
  //pinMode(buzzerPin, OUTPUT);
  //noTone(buzzerPin);
  //mq2.begin(); //TODO: This does calibration, we need to use this.

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskReadMQSensor
    ,  "ReadMQSensor"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  xTaskCreate(
    TaskRaiseAlarm
    ,  "RaiseAlarm"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  // set sytem armed to armed state as soon as system initalization is complete - TODO: This needs to be check if this is correct place
  state = ARMED;
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskReadMQSensor(void *pvParameters)  // This is a task.
{
  // only run code if system is in ARMED state
  if(state != ARMED) 
  {
    return;
  }
  
  for (;;)
  {
    smoke = analogRead(smokePin);
    if(smoke >= SMOKE_THRESHOLD_LEVEL)
    {
      // set system to ALARM state if smoke threshold is crossed.
      state = ALARM;
    }

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskRaiseAlarm(void *pvParameters)  // This is a task.
{
  // raise alarm only is system is in alarm state
  if(state != ALARM) 
  {
    return;
  }

  for (;;)
  {
    tone(buzzerPin, BUZZER_TONE);
  }
}

void armSystem()
{
    state=ARMED;
}
void disarmSystem()
{
    state=DISARMED;
}

void cancelAlarm() 
{
  // cancel alarm only works if system is in alarm state, otherwise skip
  if(state != ALARM) 
  {
    return;
  }
  
  noTone(buzzerPin);
}

void activateRedLED()  
{

}
void activateGreenLED()  
{

}

void activateYellowLED()  
{

}