/*******************************************************************************
*/
#include <stdint.h>
#include "../include/PID.h"
#include "../ssd1306.h"
//TODO: re-add included headers.

typedef enum REFLOW_STATE{

  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR

} reflow_state_t;

typedef enum REFLOW_STATUS{
  
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON

} reflow_status_t;

typedef	enum SWITCH{
 
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2

} switch_t;

typedef enum DEBOUNCE_STATE {
  
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE

}debounce_state_t;

typedef enum REFLOW_PROFILE{

  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED

} reflow_profile_t;

// ***** CONSTANTS *****
// ***** GENERAL *****
#define VERSION 2 // Replace with 1 or 2

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 200
#define TEMPERATURE_REFLOW_MAX_LF 250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 180
#define TEMPERATURE_REFLOW_MAX_PB 224
#define SOAK_MICRO_PERIOD_PB 10000

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define X_AXIS_START 18 // X-axis starting position

// ***** LCD MESSAGES *****
const char* lcd_messages_reflow_status[] = {
    "idle",
    "pre",
    "soak",
    "reflow",
    "cool",
    "finished",
    "cooling(hot)",
    "error"
};

/*
// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140, 146, 146, 140, 128, 128, 128, 128
};
*/

//TODO:set all pins to corresponding stm pins
// ***** PIN ASSIGNMENT *****
/*
uint8_t ssrPin = A0; //set this to stm analog pin 0
uint8_t fanPin = A1; //set this stm analog pin 1
uint8_t thermocoupleCSPin = 10;
uint8_t ledPin = 4;
uint8_t buzzerPin = 5;
uint8_t switchStartStopPin = 3;
uint8_t switchLfPbPin = 2;
*/
// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int32_t window_size;
uint64_t window_start_time;
uint64_t next_check;
uint64_t next_read;
uint64_t update_lcd;
uint64_t timer_soak;
uint64_t buzzer_period;
uint8_t soak_temperature_max;
uint8_t reflow_temperature_max;
uint64_t soak_micro_period;
uint64_t current_time;

// Reflow oven controller state machine state variable
reflow_state_t ReflowState;
// Reflow oven controller status
reflow_status_t ReflowStatus;
// Reflow profile type
reflow_profile_t ReflowProfile;
// Switch debounce state machine state variable
debounce_state_t DebounceState;
// Switch debounce timer
int64_t LastDebounceTime;
// Switch press status
switch_t SwitchStatus;
switch_t SwitchValue;
switch_t SwitchMask;
// Seconds timer
uint32_t timer_update;
uint32_t timer_seconds;
// Thermocouple fault status
uint8_t fault;
uint8_t temperature[SCREEN_WIDTH - X_AXIS_START];
uint8_t x;

// PID control interface
//TODO: find out which header PID is defined in, init thermocoupler




PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
// MAX31856 thermocouple interface
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(thermocoupleCSPin);


uint64_t map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void init(void*){

    //TODO: use equivalent stm for pinmode and digiwrite
    // SSR pin initialization to ensure reflow oven is off
    digitalWrite(ssrPin, LOW);
    pinMode(ssrPin, OUTPUT);

    // Buzzer pin initialization to ensure annoying buzzer is off
    digitalWrite(buzzerPin, LOW);
    pinMode(buzzerPin, OUTPUT);

    // LED pins initialization and turn on upon start-up (active high)
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);

    // Initialize thermocouple interface
    thermocouple.begin();
    thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

    // Serial communication at 115200 bps
    Serial.begin(115200);

    // Turn off LED (active high)
    digitalWrite(ledPin, LOW);
    // Set window size
    window_size = 2000;
    // Initialize time keeping variable
    next_check = HAL_GetTick();
    // Initialize thermocouple reading variable
    next_read = HAL_GetTick();
    // Initialize LCD update timer
    update_lcd = HAL_GetTick();

    loop();

}

switch_t read_switch(void){
  
  int switchAdcValue = 0;
  // Switch connected directly to individual separate pins
  if (digitalRead(switchStartStopPin) == LOW) return SWITCH_1;
  if (digitalRead(switchLfPbPin) == LOW) return SWITCH_2;

  return SWITCH_NONE;

}

uint8_t thermocoupler_error(void*){

    if ((fault & MAX31856_FAULT_CJRANGE) ||
        (fault & MAX31856_FAULT_TCRANGE) ||
        (fault & MAX31856_FAULT_CJHIGH) ||
        (fault & MAX31856_FAULT_CJLOW) ||
        (fault & MAX31856_FAULT_TCHIGH) ||
        (fault & MAX31856_FAULT_TCLOW) ||
        (fault & MAX31856_FAULT_OVUV) ||
        (fault & MAX31856_FAULT_OPEN))
    {
        reflow_state = REFLOW_STATE_ERROR;
        reflow_status = REFLOW_STATUS_OFF;
        return 1;
      // Illegal operation
    }
    
    reflow_state = REFLOW_STATE_IDLE;
    return 0;
}

void loop(){

  //bool is_running = 1;
  //while(is_running)
  //will switch while(is_running) for a while(1) loop that is interrupt driven
  // Time to read thermocouple?
  if (HAL_GetTick() > next_read) {
    // Read thermocouple next sampling period
    next_read += SENSOR_SAMPLING_TIME;
    // Read current temperature
    input = thermocouple.readThermocoupleTemperature();
    // Check for thermocouple fault
    fault = thermocouple.readFault();

    // If any thermocouple fault is detected
    if (thermocoupler_error()){
      // Illegal operation
      reflow_state = REFLOW_STATE_ERROR;
      printf("Thermocoupler fault\n");
    }
  }

  //check input
  if (HAL_GetTick() > next_check){
    // essentially we add 1000(SENSOR_SAMPLING_TIME) to next check such that we our period is millis + 1000
    next_check += SENSOR_SAMPLING_TIME;
    // If reflow process is on going
    if (reflow_status == REFLOW_STATUS_ON){
      
        // Toggle red LED as system heart beat
      digitalWrite(ledPin, !(digitalRead(ledPin)));
      // Increase seconds timer for reflow curve plot
      timer_seconds++;

    } else {
      // Turn off red LED
      digitalWrite(ledPin, LOW);
    }
  }

  //draw if appropriate
  if (HAL_GetTick() > update_lcd){
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteChar(lcd_messages_reflow_status[ReflowState], Font_6x8, White);
    ssd1306_SetCursor(115, 0);

    ssd1306_WriteChar("LF", Font_6x8, White);
    
    // Temperature markers
    ssd1306_SetCursor(0, 18);
    ssd1306_WriteChar(F("250"));
    ssd1306_SetCursor(0, 36);
    ssd1306_WriteChar(F("150"));
    ssd1306_SetCursor(0, 54);
    ssd1306_WriteChar(F("50"));
    // Draw temperature and time axis
    ssd1306_Line(18, 18, 18, 63, White);
    ssd1306_Line(18, 63, 127, 63, White);
    ssd1306_SetCursor(115, 0);

    // If currently in error state
    if (ReflowState == REFLOW_STATE_ERROR){
      ssd1306_SetCursor(80, 9);
      ssd1306_WriteChar(F("TC Error"));
    } else {
      // Right align temperature reading
      if (input < 10) ssd1306_SetCursor(91, 9);
      else if (input < 100) ssd1306_SetCursor(85,9);
      else ssd1306_SetCursor(80, 9);
      // Display current temperature
      ssd1306_WriteChar(input);
      ssd1306_WriteChar((char)247);
      ssd1306_WriteChar(F("C"));
    }
    
    if (ReflowStatus == REFLOW_STATUS_ON)
    {
      // We are updating the display faster than sensor reading
      if (timer_seconds > timer_update)
      {
        // Store temperature reading every 6 s
        if ((timer_seconds % 6) == 0)
        {
          timer_update = timer_seconds;
          uint8_t average_reading = map(input, 0, 250, 63, 19);
          if (x < (SCREEN_WIDTH - X_AXIS_START))
          {
            temperature[x++] = average_reading;
          }
        }
      }
    }
    
    for (uint8_t timeAxis = 0; timeAxis < x; timeAxis++){
      ssd1306_DrawPixel(timeAxis + X_AXIS_START, temperature[timeAxis], White);
    }
    // Update screen
    ssd1306_UpdateScreen();
  }

  // Reflow oven controller state machine
  switch (ReflowState)
  {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (input >= TEMPERATURE_ROOM){
        ReflowState = REFLOW_STATE_TOO_HOT;
        break;
      } else {
        // If switch is pressed to start reflow process
        if (SwitchStatus == SWITCH_1)
        {
          // Send header for CSV file
          // Intialize seconds timer for serial debug information
          timer_seconds = 0;
          
          // Initialize reflow plot update timer
          timer_update = 0;
          
          for (x = 0; x < (SCREEN_WIDTH - X_AXIS_START); x++){
            temperature[x] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          x = 0;
          
          // Initialize PID control window starting time
          window_start_time = HAL_GetTick();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Load profile specific constant
          soak_temperature_max = TEMPERATURE_SOAK_MAX_LF;
          reflow_temperature_max = TEMPERATURE_REFLOW_MAX_LF;
          soak_micro_period = SOAK_MICRO_PERIOD_LF;
          // Tell the PID to range between 0 and the full window size
          //pid_set_output_limits(0, window_size, pid);
          pid_set_output_limits(0, window_size, pid);
          pid_set_sample_time(PID_SAMPLE_TIME, pid);
          // Turn the PID on
          pid_set_mode(AUTOMATIC, pid);
          // Proceed to preheat stage
          ReflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      ReflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN)
      {
        // Chop soaking period into smaller sub-period
        timer_soak = HAL_GetTick() + soak_micro_period;
        // Set less agressive PID parameters for soaking ramp
        pid_set_tunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK, pid);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        ReflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (HAL_GetTick() > timer_soak)
      {
        timer_soak = HAL_GetTick() + soak_micro_period;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > soak_temperature_max)
        {
          // Set agressive PID parameters for reflow ramp
          pid_set_tunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW, 1, pid);
          // Ramp up to first section of soaking temperature
          setpoint = reflow_temperature_max;
          // Proceed to reflowing state
          ReflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (reflow_temperature_max - 5))
      {
        // Set PID parameters for cooling ramp
          pid_set_tunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW, 1, pid);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
        // Proceed to cooling state
        ReflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN)
      {
        // Retrieve current time for buzzer usage
        buzzer_period = HAL_GetTick() + 1000;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // Turn off reflow process
        ReflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        ReflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    case REFLOW_STATE_COMPLETE:
      if (HAL_GetTick() > buzzer_period)
      {
        // Turn off buzzer
        digitalWrite(buzzerPin, LOW);
        // Reflow process ended
        ReflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM)
      {
        // Ready to reflow
        ReflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_ERROR:
      thermocoupler_error(void *);
      break;
  }

  // If switch 1 is pressed
  if (SwitchStatus == SWITCH_1)
  {
    // If currently reflow process is on going
    if (ReflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      ReflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      ReflowState = REFLOW_STATE_IDLE;
    }
  }
   // Switch 2 is pressed
   // EEPROM header basically only is used for profile switching.
   /* functionality for profile switching, unneeded currently.
   * will write custom parser for custom file format here.
  else if (SwitchStatus == SWITCH_2 && ReflowState == REFLOW_STATE_IDLE)
  {
    // Only can switch reflow profile during idle
      // Currently using lead-free reflow profile
      if (reflowProfile == REFLOW_PROFILE_LEADFREE)
      {
        // Switch to leaded reflow profile
        reflowProfile = REFLOW_PROFILE_LEADED;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 1);
      }
  }
  */
  
  // Switch status has been read
  SwitchStatus = SWITCH_NONE;
  // Simple switch debounce state machine (analog switch)
  switch (DebounceState){

    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      SwitchStatus = SWITCH_NONE;
      SwitchValue = read_switch();

      // If either switch is pressed
      if (SwitchValue != SWITCH_NONE)
      {
        // Keep track of the pressed switch
        SwitchMask = SwitchValue;
        // Intialize debounce counter
        LastDebounceTime = HAL_GetTick();
        // Proceed to check validity of button press
        DebounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      SwitchValue = read_switch();
      if (SwitchValue == SwitchMask)
      {
        // If minimum debounce period is completed
        if ((HAL_GetTick() - LastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {
          // Valid switch press
          SwitchStatus = SwitchMask;
          // Proceed to wait for button release
          DebounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // False trigger
      else
      {
        // Reinitialize button debounce state machine
        DebounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      SwitchValue = read_switch();
      if (SwitchValue == SWITCH_NONE)
      {
        // Reinitialize button debounce state machine
        DebounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }

  // PID computation and SSR control
  if (ReflowStatus == REFLOW_STATUS_ON)
  {
    current_time = HAL_GetTick();
    pid_compute(pid);

    if ((current_time - window_start_time) > window_size)
    {
      // Time to shift the Relay Window
      window_start_time += window_size;
    }
    if (output > (current_time - window_start_time)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(ssrPin, LOW);
  }
}

