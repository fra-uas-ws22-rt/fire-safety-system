#include <Arduino_FreeRTOS.h>
#include <semphr.h>
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
SemaphoreHandle_t xSerialSemaphore;
enum SystemState state;
float smoke;
//MQ2 mq2(smokePin);

/**********Application Setup******************************/
void setup() {
  
  state = INIT;
  Serial.begin(9600); 

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  {
    Serial.println("Initalization started..");
    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
  }
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
    ,  3  // Priority
    ,  NULL );

  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  {
    Serial.println("Initalization Fisnhed, Setting System to ARMED state..");
    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
  }
  
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
  (void) pvParameters;
  pinMode(smokePin, INPUT);

  for (;;)
  {
    // only run code if system is in ARMED state
    if(state == ARMED) 
    {
      smoke = analogRead(smokePin);
      if(smoke >= SMOKE_THRESHOLD_LEVEL)
      {
        // set system to ALARM state if smoke threshold is crossed.
        state = ALARM;
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
          Serial.print("Alarm riase bcz smoke level is:");
          Serial.println(smoke);
          xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
        }
      }
      else
      {
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
          Serial.print("reading MQ sensor:");
          Serial.println(smoke);  
          xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
        }    
      }    

      vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
    }
  }
}

void TaskRaiseAlarm(void *pvParameters)  // This is a task.
{ 
  //pinMode(buzzerPin, OUTPUT);
  //noTone(buzzerPin); 
  
  for (;;)
  {
    if(state == ALARM) 
    {
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
         Serial.println("System is armed, running buzzer until alarm is not cancelled..");
         xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
      }
      tone(buzzerPin, BUZZER_TONE);
    }
    else
    {
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
         Serial.println("System is not armed, running buzzer skipping buzzer..");
         xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
      }
    }
    
    vTaskDelay(1);
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