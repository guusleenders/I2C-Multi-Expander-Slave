#include <Arduino.h>
#include <nI2C.h>
#include <avr/wdt.h>
#include <EEPROM.h>

/*
TODO: 
- Digital inputs debouncing (and on interrupt)
- Testing
  Done: Analog input, Digital input, Digital output, frequency
*/

#define REGISTER_MAP_SIZE                       0xA0
#define REGISTER_RESPONSE_SIZE                  32

// Start register map
#define REGISTER_START_SETTINGS                 0x00
#define REGISTER_START_ANALOG_OUTPUT            0x10
#define REGISTER_START_ANALOG_INPUT             0x20   // 2 byte register
#define REGISTER_START_DIGITAL_OUTPUT           0x40
#define REGISTER_START_DIGITAL_INPUT            0x50
#define REGISTER_START_DIGITAL_INPUT_FREQUENCY  0x60   // 4 byte register, in pulses per DEFAULT_FREQUENCY_COUNTER_TIME

// End of register map
#define REGISTER_END_SETTINGS                   REGISTER_START_ANALOG_OUTPUT
#define REGISTER_END_ANALOG_OUTPUT              REGISTER_START_ANALOG_INPUT
#define REGISTER_END_ANALOG_INPUT               REGISTER_START_DIGITAL_OUTPUT
#define REGISTER_END_DIGITAL_OUTPUT             REGISTER_START_DIGITAL_INPUT
#define REGISTER_END_DIGITAL_INPUT              REGISTER_START_DIGITAL_INPUT_FREQUENCY
#define REGISTER_END_DIGITAL_INPUT_FREQUENCY    REGISTER_MAP_SIZE

// Settings registers
#define REGISTER_SETTINGS_DEVICE_ID             0x00
#define REGISTER_SETTINGS_HARDWARE_VERSION      0x01
#define REGISTER_SETTINGS_FIRMWARE_VERSION      0x02
#define REGISTER_SETTINGS_POLLING_TIME          0x05
#define REGISTER_SETTINGS_DEBOUNCE_TIME         0x06  // Not implemented
#define REGISTER_SETTINGS_PWM_FREQUENCY         0x07  // Not implemented
#define REGISTER_SETTINGS_FREQUENCY_COUNTER_TIME 0x08 // In seconds
#define REGISTER_SETTINGS_FREQUENCY_DETECTION   0x09

// Default values of settings registers
#define DEVICE_ID                               0x85
#define HARDWARE_VERSION                        0x01
#define FIRMWARE_VERSION                        0x01
#define DEFAULT_POLLING_TIME                    50  // In milliseconds
#define DEFAULT_DEBOUNCE_TIME                   50  // In milliseconds, not implemented
#define DEFAULT_PWM_FREQUENCY                   0
#define DEFAULT_FREQUENCY_COUNTER_TIME          5   // In seconds
#define DEFAULT_FREQUENCY_DETECTION             RISING

// Pin definitions
#define ANALOG_OUTPUT_PINS                      0
#define ANALOG_INPUT_PINS                       4
#define DIGITAL_OUTPUT_PINS                     3
#define DIGITAL_INPUT_PINS                      3
#define DIGITAL_INPUT_FREQUENCY_PINS            2

#define EEPROM_STATUS_ADDRESS                   0
#define EEPROM_START_ADDRESS                    1

const uint8_t analog_output_pins[] = {};
const uint8_t analog_input_pins[] = {A0, A1, A2, A3};
const uint8_t digital_output_pins[] = {9, 6, 5};
const uint8_t digital_input_pins[] = {2, 3, 4};
const uint8_t digital_input_frequency_pins[] = {2, 3};

enum address_i2c_t : byte{
    ADDRESS_I2C = 0x2F // Address of I2C device
};

// Global TWI object
CTWI i2c;

uint8_t registerMap[REGISTER_MAP_SIZE] = {0x00};
bool registerMapUpdate = true;
bool registerMapSettingsUpdate = false;
uint8_t lastRegister = 0;
uint32_t lastPollTime = 0;
volatile uint32_t pulseCount[DIGITAL_INPUT_FREQUENCY_PINS] = {0};
volatile bool pulseState[DIGITAL_INPUT_FREQUENCY_PINS] = {false};
uint32_t lastFrequencyTime = 0;

// Callback function prototype
void i2cWriteCallback(const uint8_t data[], const uint8_t length);
void i2cReadCallback(void);
void countFrequencyISR(void);
void attachInterrupts(void);
void detachInterrupts(void);
void readEEPROM(void);
void updateEEPROM(void);

void setup() {
  // Assign slave address
  i2c.SetLocalDeviceAddress(ADDRESS_I2C);
  
  // Set read and write callback
  i2c.SetSlaveReceiveHandler(i2cWriteCallback);
  i2c.SetSlaveTransmitHandler(i2cReadCallback);

  // Load config from EEPROM
  
  if(EEPROM.read(EEPROM_STATUS_ADDRESS) == 1){
    readEEPROM();
  }else{
    registerMap[REGISTER_SETTINGS_DEVICE_ID] = DEVICE_ID;
    registerMap[REGISTER_SETTINGS_HARDWARE_VERSION] = HARDWARE_VERSION;
    registerMap[REGISTER_SETTINGS_FIRMWARE_VERSION] = FIRMWARE_VERSION;
    registerMap[REGISTER_SETTINGS_POLLING_TIME] = DEFAULT_POLLING_TIME;
    registerMap[REGISTER_SETTINGS_DEBOUNCE_TIME] = DEFAULT_DEBOUNCE_TIME;
    registerMap[REGISTER_SETTINGS_PWM_FREQUENCY] = DEFAULT_PWM_FREQUENCY;
    registerMap[REGISTER_SETTINGS_FREQUENCY_COUNTER_TIME] = DEFAULT_FREQUENCY_COUNTER_TIME;
    registerMap[REGISTER_SETTINGS_FREQUENCY_DETECTION] = DEFAULT_FREQUENCY_DETECTION;

    updateEEPROM();
  }

  
  
  
  // Analog pins do not need init
  // Init digital outputs
  for(uint8_t pin = 0; pin < DIGITAL_OUTPUT_PINS; pin++){
    pinMode(digital_output_pins[pin], OUTPUT);
  }
  // Init digital inputs
  for(uint8_t pin = 0; pin < DIGITAL_INPUT_PINS; pin++){
    pinMode(digital_input_pins[pin], INPUT);
  }
  // Init digital inputs for frequency counting
  for(uint8_t pin = 0; pin < DIGITAL_INPUT_FREQUENCY_PINS; pin++){
    pinMode(digital_input_frequency_pins[pin], INPUT);
    attachInterrupt(digitalPinToInterrupt(digital_input_frequency_pins[pin]), countFrequencyISR, CHANGE);
  }
  lastFrequencyTime = millis();

  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();

  if(registerMapUpdate){
    // Go over outputs to put them in the right state

    // Analog outputs
    // Not implemented right now, this is done in another i2c device
    
    // Digital outputs
    for(uint8_t pin = 0; pin < DIGITAL_OUTPUT_PINS; pin++){
      // These can be pwm devices, analogWrite is anything above 128 = high
      uint8_t reg = REGISTER_START_DIGITAL_OUTPUT | pin;
      
      analogWrite(digital_output_pins[pin], registerMap[reg]);
    }
    registerMapUpdate = false;
  }

  if(registerMapSettingsUpdate){
    updateEEPROM();
  }

  if(millis() - lastPollTime > registerMap[REGISTER_SETTINGS_POLLING_TIME]){
    // Analog inputs
    for(uint8_t pin = 0; pin < ANALOG_INPUT_PINS; pin++){
      uint8_t reg = REGISTER_START_ANALOG_INPUT | (pin << 1);
      uint16_t analogValue = analogRead(analog_input_pins[pin]);
      registerMap[reg+1] = analogValue & 0x00FF;
      registerMap[reg] = (analogValue & 0xFF00) >> 8;
    }

    // Digital inputs
    for(uint8_t pin = 0; pin < DIGITAL_INPUT_PINS; pin++){
      uint8_t reg = REGISTER_START_DIGITAL_INPUT | pin;
      registerMap[reg] = digitalRead(digital_input_pins[pin]);
      
      
    }

    lastPollTime = millis();
  }

  if(millis() - lastFrequencyTime > registerMap[REGISTER_SETTINGS_FREQUENCY_COUNTER_TIME]*1000){  

    noInterrupts();
    for(uint8_t pin = 0; pin < DIGITAL_INPUT_FREQUENCY_PINS; pin++){
      uint8_t reg = REGISTER_START_DIGITAL_INPUT_FREQUENCY | (pin << 2);
      // Rescale to REGISTER_SETTINGS_FREQUENCY_COUNTER_TIME because this may not be exact
      uint32_t frequencyValue = (float(registerMap[REGISTER_SETTINGS_FREQUENCY_COUNTER_TIME]*1000.0)/(millis() - lastFrequencyTime))*pulseCount[pin];
      
      registerMap[reg+3] =  frequencyValue & 0x000000FF;
      registerMap[reg+2] = (frequencyValue & 0x0000FF00) >> 8;
      registerMap[reg+1] = (frequencyValue & 0x00FF0000) >> 16;
      registerMap[reg] =   (frequencyValue & 0xFF000000) >> 24;

      pulseCount[pin] = 0;

    }
    interrupts();
    lastFrequencyTime = millis();
  }
}

void readEEPROM(){
  for(uint8_t reg = REGISTER_START_SETTINGS; reg < REGISTER_END_SETTINGS; reg++){
    registerMap[reg] = EEPROM.read(reg - REGISTER_START_SETTINGS + EEPROM_START_ADDRESS);
  }
}

void updateEEPROM(){
  EEPROM.update(EEPROM_STATUS_ADDRESS, 1);
  for(uint8_t reg = REGISTER_START_SETTINGS; reg < REGISTER_END_SETTINGS; reg++){
    EEPROM.update(reg - REGISTER_START_SETTINGS + EEPROM_START_ADDRESS, registerMap[reg]);
  }
}

void attachInterrupts(){
  for(uint8_t pin = 0; pin < DIGITAL_INPUT_FREQUENCY_PINS; pin++){
    attachInterrupt(digitalPinToInterrupt(digital_input_frequency_pins[pin]), countFrequencyISR, CHANGE);
  }
}

void detachInterrupts(){
  for(uint8_t pin = 0; pin < DIGITAL_INPUT_FREQUENCY_PINS; pin++){
    detachInterrupt(digitalPinToInterrupt(digital_input_frequency_pins[pin]));
  }
}

void i2cWriteCallback(const uint8_t data[], const uint8_t length){
  detachInterrupts();
  if(data[0] < REGISTER_MAP_SIZE){
    // Buffer this register so we can answer to a read request later
    lastRegister = data[0];

    // If more bytes are transferred: write operation
    if(length > 1){
      memcpy(&registerMap[lastRegister], &data[1], length-1);
      registerMapUpdate = true;
      // Refresh EEPROM when we wrote things to settings registers
      if(lastRegister < REGISTER_END_SETTINGS || lastRegister + length < REGISTER_END_SETTINGS ){
        registerMapSettingsUpdate = true;
      }
    }
  }
  attachInterrupts();
}

void i2cReadCallback(void){
  // Send message to master
  detachInterrupts();

  uint8_t size = REGISTER_RESPONSE_SIZE; 
  if(lastRegister + REGISTER_RESPONSE_SIZE > REGISTER_MAP_SIZE){
    size = REGISTER_MAP_SIZE - lastRegister;
  } 

  i2c.SlaveQueueNonBlocking(registerMap + lastRegister, size);

  attachInterrupts();
}

void countFrequencyISR(void){
  detachInterrupts();

  for(uint8_t pin = 0; pin < DIGITAL_INPUT_FREQUENCY_PINS; pin++){
    bool stateNow = digitalRead(digital_input_frequency_pins[pin]);
    // Rising edge detection
    if(registerMap[REGISTER_SETTINGS_FREQUENCY_DETECTION] == RISING){
      if(stateNow > pulseState[pin]){
        pulseCount[pin]++;
      }
    }else if(registerMap[REGISTER_SETTINGS_FREQUENCY_DETECTION] == FALLING){
      if(stateNow < pulseState[pin]){
        pulseCount[pin]++;
      }
    }else if(registerMap[REGISTER_SETTINGS_FREQUENCY_DETECTION] == CHANGE){
      if(stateNow != pulseState[pin]){
        pulseCount[pin]++;
      }
    }
    pulseState[pin] = stateNow;
  }

  attachInterrupts();
}
