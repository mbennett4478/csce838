#include <Arduino.h>

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

long previousBMillis = 0;
long previousYMillis = 0;

long interval = 1000;

void startTimer(int frequencyHz);
void setTimerYFrequency(int frequencyHz);
void setTimerBFrequency(int frequencyHz);
void TC3_Handler();
void taskOne();
void taskTwo();
void taskThree();
void tc3Setup(int frequencyHz);
void tc4Setup(int frequencyHz);

bool isLEDOn = false;
uint16_t timeCounter = 0; 

void setup() {
  SerialUSB.begin(9600);
  pinMode(PIN_LED_13, OUTPUT);
  pinMode(PIN_LED_RXL, OUTPUT);
  startTimer(1);
}

void loop() {
  // taskOne();
}

void taskOne() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousBMillis > interval) {
    previousBMillis = currentMillis;
    digitalWrite(PIN_LED_13, !digitalRead(PIN_LED_13));
    SerialUSB.println("Blue LED is " + digitalRead(PIN_LED_13) ? "on" : "off");
  }

  if (currentMillis - previousYMillis > interval/3) {
    previousYMillis = currentMillis;
    digitalWrite(PIN_LED_RXL, !digitalRead(PIN_LED_RXL));
    SerialUSB.println("Yellow LED is " + digitalRead(PIN_LED_RXL) ? "on" : "off");
  }
}

void taskTwo() {

}

void setTimerBFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  SerialUSB.println(TC->COUNT.reg);
  SerialUSB.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void setTimerYFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC4;
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  SerialUSB.println(TC->COUNT.reg);
  SerialUSB.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  tc3Setup(1);
  // tc4Setup(1);
}

void tc3Setup(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (
    GCLK_CLKCTRL_CLKEN | 
    GCLK_CLKCTRL_GEN_GCLK0 | 
    GCLK_CLKCTRL_ID_TCC2_TC3
  );

  while (GCLK->STATUS.bit.SYNCBUSY == 1);

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while(TC->STATUS.bit.SYNCBUSY == 1);
  
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while(TC->STATUS.bit.SYNCBUSY == 1);

  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while(TC->STATUS.bit.SYNCBUSY == 1);

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while(TC->STATUS.bit.SYNCBUSY == 1);

  // setTimerBFrequency(frequencyHz);
  setTimerBFrequency(frequencyHz * 3);

  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while(TC->STATUS.bit.SYNCBUSY == 1);
}

void tc4Setup(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | 
    GCLK_CLKCTRL_GEN_GCLK0 | 
    GCLK_CLKCTRL_ID_TC4_TC5
  );

  while (GCLK->STATUS.bit.SYNCBUSY == 1);

  TcCount16* TC = (TcCount16*) TC4;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while(TC->STATUS.bit.SYNCBUSY == 1);
  
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while(TC->STATUS.bit.SYNCBUSY == 1);

  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while(TC->STATUS.bit.SYNCBUSY == 1);

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while(TC->STATUS.bit.SYNCBUSY == 1);

  setTimerYFrequency(frequencyHz*3);

  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC4_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while(TC->STATUS.bit.SYNCBUSY == 1);
}

void TC3_Handler() {
  // TcCount16* TC = (TcCount16*) TC3;

  // if (TC->INTFLAG.bit.MC0 == 1) {
  //   TC->INTFLAG.bit.MC0 = 1;
  //   // Write callback here!!!
  //   digitalWrite(PIN_LED_13, !digitalRead(PIN_LED_13));
  // }
  taskThree();
}

void TC4_Handler() {
  TcCount16* TC = (TcCount16*) TC4;

  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    digitalWrite(PIN_LED_RXL, !digitalRead(PIN_LED_RXL));
  }
}

void taskThree() {
  TcCount16* TC = (TcCount16*) TC3;

  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    timeCounter++;

    if (timeCounter % 3 == 0) {
      digitalWrite(PIN_LED_13, !digitalRead(PIN_LED_13));
    }

    digitalWrite(PIN_LED_RXL, !digitalRead(PIN_LED_RXL));
  }
}
