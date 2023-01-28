#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Keypad.h>
#include <task.h>

/**********Decalarations of Tasks************************/
void TaskBlink( void *pvParameters );
void TaskReadMQSensor( void *pvParameters );
void TaskRaiseAlarm( void *pvParameters );
void TaskReadKeypad( void *pvParameters );

/**********Decalarations of Functions********************/
void setSystemState();
void armSystem();
void disarmSystem();
void cancelAlarm();
void activateRedLED();
void activateGreenLED();
void activateYellowLED();

/**********Decalarations of Hardware Pins****************/
int smokePin = A0;
int buzzerPin = 13;

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
#define   BUZZER_TONE             1000
#define   SYSTEM_DISARM_CODE      "1234"
#define   SYSTEM_ARM_CODE         "4321"
#define   SYSTEM_CANCEL_ALARM     "0000"

/**********Decalarations of Global Varaibles**************/
SemaphoreHandle_t xSerialSemaphore = NULL;
SemaphoreHandle_t xStateSemaphore = NULL;
enum SystemState state;
float smoke;
const byte ROWS = 4; 
const byte COLS = 4; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6}; 
byte colPins[COLS] = {5, 4, 3, 2}; 

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 


/**********Application Setup******************************/
void setup() {
  
  state = INIT;
  Serial.begin(9600); 
  pinMode(smokePin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);

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

  xTaskCreate(
    TaskReadKeypad
    ,  "ReadKeypad"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  {
    Serial.println("Initalization Fisnhed, Setting System to ARMED state..");
    xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
  }
  
  //set sytem armed to armed state as soon as system initalization is complete - TODO: This needs to be check if this is correct place
  state = ARMED;
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskReadMQSensor(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
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
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 1 ) == pdTRUE )
        {
          Serial.print("Alarm riase bcz smoke level is: ");
          Serial.println(smoke);
          xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
        }
      }
      else
      {
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 1 ) == pdTRUE )
        {
          Serial.print("reading MQ sensor: ");
          Serial.println(smoke);  
          xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
        }    
      }    
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskRaiseAlarm(void *pvParameters)  // This is a task.
{
  for (;;)
  {
    if(state == ALARM) 
    {
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 1 ) == pdTRUE )
      {
         Serial.println("System is armed, running buzzer..");
         xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
      }
      digitalWrite(buzzerPin, LOW);
    }
    else
    {
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 1 ) == pdTRUE )
      {
         Serial.println("System not armed, skipping buzzer..");
         xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
      }
      digitalWrite(buzzerPin, HIGH);
    }
    vTaskDelay(1);
  }
}

void TaskReadKeypad(void *pvParameters)  // This is a task.
{
  for (;;) {
    char key = customKeypad.getKey();
    if (key == 'D') {
      if (xSemaphoreTake(xSerialSemaphore, (TickType_t)1) == pdTRUE) {
        Serial.println("D Pressed,.");
        xSemaphoreGive(xSerialSemaphore);  // Now free or "Give" the Serial Port for others.
      }
      disarmSystem();
    } else if (key == 'A') {
      if (xSemaphoreTake(xSerialSemaphore, (TickType_t)1) == pdTRUE) {
        Serial.println("A Pressed,..");
        xSemaphoreGive(xSerialSemaphore);  // Now free or "Give" the Serial Port for others.
      }
      armSystem();
    } else if (key == 'C') {
      if (xSemaphoreTake(xSerialSemaphore, (TickType_t)1) == pdTRUE) {
        Serial.println("C Pressed, ..");
        xSemaphoreGive(xSerialSemaphore);  // Now free or "Give" the Serial Port for others.
      }
      cancelAlarm();
    } else if (key == 'B') {
      if (xSemaphoreTake(xSerialSemaphore, (TickType_t)1) == pdTRUE) {
        Serial.println("B Pressed, ..");
        xSemaphoreGive(xSerialSemaphore);  // Now free or "Give" the Serial Port for others.
      }
      state = ALARM;
    }
    //vTaskDelay(1);
  }
}

void armSystem() {
  state = ARMED;
}

void disarmSystem() {
  digitalWrite(buzzerPin, HIGH);
  state = DISARMED;
}

void cancelAlarm() {
  // cancel alarm only works if system is in alarm state, otherwise skip
  if (state != ALARM) {
    return;
  }

  armSystem();
  digitalWrite(buzzerPin, HIGH);
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