// Temperature controller for cigarette, schematics in ard-smoke.sch
//
//    Copyright (C) 2015  Vasim V.
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public License
//    along with this program. 

#include <U8glib.h>
#include <MsTimer2.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <avr/sleep.h>

// Additional info to console
// #define DEBUG

// Temperature ADC pin - ITEMP
#define P_ITEMP 0
// Battery ADC pin - CHECK
#define P_VMAIN 1
// Coil N-FET gate - OOn
#define P_OON 9
// Temperature check N-FET gate - OTest
#define P_OTEST 3

#define NBUTTONS 3
// Coil button
#define P_BCOIL 10
// Plus button */
#define P_BPLUS 11
// Minus button */
#define P_BMINUS 2

// Delay for FET change, microseconds
#define FETSW_DELAY 20

// Interrupt period (timer2), microseconds
#define TIMER_PERIOD 10000

// PWM Period, microseconds
#define PWM_PERIOD 1024

// Test resistor resistance, in Ohm
#define RTEST 4.7f

// Temparture-resistance koefficient
// Titan
#define RTCHANGE 0.003525f
// 316L
// #define RTCHANGE 0.000879f
// 304
// #define RTCHANGE 0.001016f
// NiFE30
// #define RTCHANGE 0.003200f
// Kantal - not working (too low resolution)
// #define RTCHANGE 0.0001f

// Minium coil resistance, in Ohm
#define MIN_RCOIL 0.2f

// Warning battery voltage (will flash on screen)
#define WARN_VOLTAGE 6.6f
// Minimum battery voltage (will not fire coil if lower)
#define MIN_VOLTAGE 6.2f

// Default zero (when no difference between rcoil_zero and rcoil) temperature
#define TEMP_ZERO 25

// Use atmega's internal temperature sensor (will be calibrated with TEMP_ZERO at startup)
// Doesn't work on atmega8 and atmega168
#define USE_INTERNAL_TEMP

// Sleep timer (in timer2 cycles, seconds*100), activate if coil button was pressed for
#define SLEEP_TIME 12000

// How long you should keep button pressed to change temperature, in time2 cycles (seconds*100)
#define BUTTON_SENS_TIME 100

// Quite crude numbers. TODO: implement an auto-tune
#define PID_P 70.0
#define PID_I 0.75
#define PID_D 20.0

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);

// Coil is on or not
boolean coil_on;
// Voltage readings from ADCs
int coilv;
int coilv_prev;
int vmainv;
int vmainv_prev;
// Calculated from vmainv, in volts
float vbat;
// Calculated from coilv, in volts and ohms
float voff;
float rcoil;

// Coil resistance at 25C
float rcoil_zero = 0.0f;

// Current temperature in celsius
double tcur;

// Cut-off temperature in celsius
double tcut;

// Outside temperature
double tair;

// PID regulator stuff
double Output;

// Buttons debounce stuff
#define I_BCOIL 0
#define I_BPLUS 1
#define I_BMINUS 2
uint8_t pbuttons[NBUTTONS] = {P_BCOIL, P_BPLUS, P_BMINUS};
uint8_t sw_check[NBUTTONS];

// Buttons state after debouncing
boolean sbuttons[NBUTTONS] = {false, false, false};

// How long buttons state wasn't changed (in timer2 cycles)
uint32_t idlebuttons[NBUTTONS] = {0, 0, 0};

// Lock sleep mode after pin change interrupt (minus button)
volatile int lock_sleep = 0;

// Ignore if buttons were pressed at startup
boolean ignore_buttons = false;

// In deep sleep
boolean sleeping;

double errSum, lastErr;
double outputLast = 0.0;
#define MAX_CHANGE 1023.0

void MyPIDStart() {
  errSum = lastErr = outputLast = 0.0;
  MyPIDCompute();
} // MyPIDStart

// Calculate output
void MyPIDCompute()
{
   double timeChange = TIMER_PERIOD / 1000;
   double error = tcut - tcur;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
   double preOutput;
  
   /*Compute PID Output*/
   preOutput = PID_P * error + PID_I * errSum + PID_D * dErr;
   if ((preOutput - outputLast) > MAX_CHANGE)
     preOutput = outputLast + MAX_CHANGE;
   else if ((outputLast - preOutput) > MAX_CHANGE)
     preOutput = outputLast - MAX_CHANGE;   
   Output = preOutput - outputLast;
   if (Output < 0)
     Output = 0;
  
   /*Remember some variables for next time*/
   outputLast = preOutput;
   lastErr = error;
} // MyPIDCompute

// Calculate coil resistance, temperature and voltage
void update_stuff() {
  if (coilv_prev <= 0)
    coilv_prev = coilv;
  if (vmainv_prev <= 0)
    vmainv_prev = vmainv;
  rcoil = (((vmainv + vmainv_prev) / 2.0f) * RTEST) / (((coilv + coilv_prev) / 2.0f) * 1.0f + 1) - RTEST;
  if (rcoil < 0)
    rcoil = 0;
  if (rcoil_zero > 0.01) 
    tcur = double((rcoil / rcoil_zero - 1) / RTCHANGE) + tair;
  else
    tcur = tair;
  vmainv_prev = vmainv;
  coilv_prev = coilv;
  vbat = (10.0 * vmainv) / 1024.0;
} // update_temp

// Write float into eeprom
void eeprom_writef(int addr, float x) {
  uint8_t i;

  for (i = 0; i < sizeof(float); i++) {
    EEPROM.update(addr + i, *((char *) &x + i));
  }
} // eeprom_writef

// Read float from eeprom
float eeprom_readf(int addr) {
  float x;
  int i;

  for (i = 0; i < sizeof(float); i++) {
    *((char *) &x + i) = EEPROM.read(addr + i);
  }
  // Test if we did read float really, not "NAN"
  if ((x+1.0) > x)
    return x;
  return 0;
} // eeprom_readf

// Debounce buttons
void debounce_buttons() {
  uint8_t i;

  for (i = 0; i < NBUTTONS; i++) {
    if (sbuttons[i] != (digitalRead(pbuttons[i]) == LOW)) {
      if (sw_check[i] > 5) {
        sw_check[i] = 0;
        sbuttons[i] = (digitalRead(pbuttons[i]) == LOW);
        idlebuttons[i] = 0;
      } else {
        sw_check[i]++;
        idlebuttons[i]++;
      }
    } else {
      sw_check[i] = 0;
      idlebuttons[i]++;
    }
  }
} // debounce_buttons

// Update lock_sleep counter
void locksleep_update() {
  if (lock_sleep > 0)
    lock_sleep--;
  if (lock_sleep < 0)
    lock_sleep = 0;
} // locksleep_update

// Force turn off coil
void turn_off_coil() {
  Timer1.disablePwm(P_OON);
  digitalWrite(P_OON, LOW);
  delayMicroseconds(FETSW_DELAY);
} // turn_off_coil

// Turn on coil with PWM
void turn_on_coil() {
  Timer1.pwm(P_OON, Output, PWM_PERIOD);
}

// Measure temperature and voltage
void measure_stuff() {
  digitalWrite(P_OTEST, HIGH);
  delayMicroseconds(FETSW_DELAY);
  coilv = (analogRead(P_ITEMP) + analogRead(P_ITEMP)) / 2;
  // delayMicroseconds(FETSW_DELAY);
  vmainv = (analogRead(P_VMAIN) + analogRead(P_VMAIN)) / 2;
  digitalWrite(P_OTEST, LOW);
  delayMicroseconds(FETSW_DELAY);
} // measure_stuff

// Check failsafe values (battery voltage, coil resistance)
boolean check_failsafes() {
  return (vbat > MIN_VOLTAGE) && (rcoil > MIN_RCOIL);
}

#ifdef USE_INTERNAL_TEMP
// internal temperature sensor calibration
double tint_offset;

// Get internal temperature sensor, code from arduino playground site
double get_internal_temp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC
  delay(20);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;
  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;
  // Reset reference voltage
  analogReference(DEFAULT);
  delay(20);
  analogRead(P_ITEMP);
  return (t);
} // get_internal_temp
#endif

static int loopcycle = 0;
static int showtemp;
static float showrcoil;

// D2 change interrupt call - lock sleep mode for 2 seconds
void wake_up_check() {
  lock_sleep = 300;
} // wake_up_check

float vbat_show;

// u8glib draw function
void draw() {
  char tmpbuf[16];

  // Don't draw if we're going into sleep mode
  if (!sleeping && idlebuttons[I_BCOIL] > SLEEP_TIME)
    return;
    
  u8g.setFont(u8g_font_helvR12);

  // Battery voltage, flashing if too low
  memcpy(tmpbuf, "Batt", 4);
  if (vmainv < 1022) {
    dtostrf(vbat, 4, 1, tmpbuf+4);
    memcpy(tmpbuf+8, "V - ", 4);
  } else
    memcpy(tmpbuf+4, " MAX", 5);

  if (vbat > 6.3) 
    memcpy(tmpbuf+12, "OK", 3);
  else {
    if (loopcycle %  2)
      u8g.setFont(u8g_font_helvB12);
    memcpy(tmpbuf+12, "LOW!", 5);
  }
  u8g.drawStr(0, 64, tmpbuf);
  u8g.setFont(u8g_font_helvR12);
  
  // Buttons states (right top corner)
  if (sbuttons[I_BCOIL])
    u8g.drawStr(90, 16, "C");
  else
    u8g.drawStr(90, 16, ".");

  if (sbuttons[I_BPLUS])
    u8g.drawStr(99, 16, "+");
  else
    u8g.drawStr(99, 16, ".");

  if (sbuttons[I_BMINUS])
    u8g.drawStr(108, 16, "-");
  else
    u8g.drawStr(108, 16, ".");

  // Show temperature limit
  memcpy(tmpbuf, "= ", 3);
  dtostrf(tcut, 4, 0, tmpbuf+2);
  u8g.drawStr(0, 48, tmpbuf);
  
  // Show coil resistance
  tmpbuf[0] = 'R';
  dtostrf(showrcoil, 5, 2, tmpbuf+1);
  memcpy(tmpbuf+6, "O", 2);
  u8g.drawStr(0, 32, tmpbuf);

  // Show internal temperature
  dtostrf(tair, 3, 0, tmpbuf);
  tmpbuf[3] = 'C';
  tmpbuf[4] = '\0';
  u8g.drawStr(81, 32,tmpbuf);
  
  // Show current temperature and coil state
  dtostrf(float(showtemp), 4, 0, tmpbuf);
  tmpbuf[4] = 'C';
  tmpbuf[5] = '\0';
  u8g.drawStr(18, 16, tmpbuf);
  if (coil_on)
    u8g.drawStr(0, 16, ">>");
  else
    u8g.drawStr(0, 16, "  ");

  // Show current output level (quite useless, too fast change)
  dtostrf(float(Output), 5, 0, tmpbuf);
  u8g.drawStr(64, 48, tmpbuf);
} // draw

// To test if minus button was pressed 3 times in 6 seconds - exit sleep mode
boolean last_b_state;
int nchanges;
int ncycles;

// Update EEPROM with temperature limit 
boolean need_update_eeprom = false;

// Change temperature limit if MINUS or PLUS pressed
void change_temp() {
  if (!ignore_buttons) {
    if (sbuttons[I_BMINUS] && idlebuttons[I_BMINUS] > BUTTON_SENS_TIME) {
      tcut = tcut - 5;
      idlebuttons[I_BMINUS] = 0;
      need_update_eeprom = true;
    }
    if (sbuttons[I_BPLUS] && idlebuttons[I_BPLUS] > BUTTON_SENS_TIME) {
      tcut = tcut + 5;
      idlebuttons[I_BPLUS] = 0;
      need_update_eeprom = true;
    }
  }
} // change_temp

// Update EEPROM if needed
void update_eeprom() {
  if (need_update_eeprom) {
    eeprom_writef(0, rcoil_zero);
    eeprom_writef(4, tcut);
    need_update_eeprom = false;
  }
} // update_eeprom

// Prepare sleep mode (turn of screen, etc)
void prepare_sleep() {
#ifdef DEBUG
  Serial.println("Preparing sleep mode");
#endif
  sleeping = true;
  ignore_buttons = true;
  u8g.sleepOn();
  // Make sure the display receives sleep command
  delay(100);
} // prepare_sleep

// Entering sleep mode
void entering_sleep() {
  ignore_buttons = true;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  attachInterrupt(0,wake_up_check, CHANGE);
  sleep_enable();
  sleep_mode();
  // Return here after MINUS pressed
} // entering sleep

// Wake up
void exit_sleep() {
  int i;

  rcoil_zero = 0;
  sleeping = false;
  u8g.sleepOff();
  for (i = 0; i < NBUTTONS; i++)
    idlebuttons[i] = 0;
#ifdef USE_INTERNAL_TEMP
  tair = get_internal_temp() + tint_offset;
#endif
} // exit_sleep

// un-ignore buttons after startup with pressed
void unignore_buttons() {
  if (ignore_buttons && !sleeping) {
    int i;
    boolean pressed = false;

    for (i = 0; i < NBUTTONS; i++) {
      if (sbuttons[i])
        pressed = true;
    }
    if (!pressed)
      ignore_buttons = false;
  }
} // unignore_buttons

// Check if we should go out of sleep mode
void check_unsleep() {
  ncycles++;
  if (last_b_state != sbuttons[I_BMINUS]) {
    ncycles = 0;
    last_b_state = sbuttons[I_BMINUS];
    nchanges++;
    if (nchanges >= 6) {
#ifdef DEBUG
      Serial.println("Waking up");
#endif
      nchanges = 0;
      ncycles = 0;
      exit_sleep();
    }
  } else {
    if (ncycles > 200) {
      ncycles = 0;
      nchanges = 0;
    }
  }
} // check_unsleep

void TIMER2_intcallback();


// Main arduino setup (reset/poweron)
void setup() {
  Serial.begin(115200);

  ignore_buttons = true;
  sleeping = false;
  // ADC pins
  pinMode(P_ITEMP, INPUT);
  pinMode(P_VMAIN, INPUT);
  // Coil/check FETs
  pinMode(P_OON, OUTPUT);
  digitalWrite(P_OON, LOW);
  coil_on = false;
  pinMode(P_OTEST, OUTPUT);
  digitalWrite(P_OTEST, LOW);
  // Configuring buttons pins
  pinMode(P_BCOIL, INPUT_PULLUP);
  pinMode(P_BPLUS, INPUT_PULLUP);
  pinMode(P_BMINUS, INPUT_PULLUP);

  // Display init
  // Rotate if needed
  // u8g.setRot180();
  // From u8g_lib example - assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }

  // Read coil resistance at 25C from eeprom, update current values
//  rcoil_zero = eeprom_readf(0);
  tcut = eeprom_readf(4);
  if ((tcut < 100.0) || (tcut > 300.0))
    tcut = 200;

  tair = TEMP_ZERO;
#ifdef USE_INTERNAL_TEMP
  tint_offset = TEMP_ZERO - get_internal_temp();
#endif

  // Disable buttons until released if any of it pressed at startup
  if ((digitalRead(P_BPLUS) == LOW) || (digitalRead(P_BMINUS) == LOW) || (digitalRead(P_BCOIL) == LOW)) {
    int i;

    ignore_buttons = true;
    for (i = 0; i < NBUTTONS; i++)
      sbuttons[i] = (digitalRead(pbuttons[i]) == LOW);
  } else
    ignore_buttons = false;

  // Timers initialize
  MsTimer2::set(TIMER_PERIOD / 1000, TIMER2_intcallback);
  MsTimer2::start();

  Timer1.initialize(1000);
  Timer1.pwm(9, 0, PWM_PERIOD);
} // setup


// Timer interrupt - Test buttons, check temperature and fire coil, etc
void TIMER2_intcallback() {
  uint8_t i;

  debounce_buttons();

  locksleep_update();

  // Measuring temperature - turning off main coil FET before
  if (coil_on || ignore_buttons) {
    turn_off_coil();
  }

  // Don't do anything else in sleep mode
  if (sleeping || ignore_buttons)
    return;

  // Measure and update output from PID controller
  measure_stuff();

  // Measuring is done, turning on coil back
  if (coil_on)
    turn_on_coil();
  else
    turn_off_coil();

  update_stuff();

  // PID processing (calculate output)
  if (coil_on)
    MyPIDCompute();

#ifdef DEBUG
  Serial.print("Temp: ");
  Serial.print(tcur+TEMP_ZERO);
  Serial.print("C, rcoil: ");
  Serial.print(rcoil);
  Serial.print("Ohm, Output: ");
  Serial.println(Output);
#endif

  // Fire coil if button pressed
  if (!coil_on && sbuttons[I_BCOIL] && !ignore_buttons && (tcut >= tcur) && check_failsafes()) {
    MyPIDStart();
    coil_on = true;
    turn_on_coil();
  }

  // Coil shutdown if button is released or more than 5 celsius above or failsafes
  if (!sbuttons[I_BCOIL] || ((tcur - 5) > tcut) || !check_failsafes()) {
    coil_on = false;
    Output = 0;
    turn_off_coil();
  }

  // Switch ADC MUX
  analogRead(P_ITEMP);
} // TIMER2_intcallback


// Main arduino loop
void loop() {
  int i;
  
  vbat_show = vbat;
  voff = (10.0 * coilv) / 1024.0;
  showtemp = tcur;
  showrcoil = rcoil;

  // Redraw screen
  if (!sleeping) {
    // u8g_lib picture loop
    u8g.firstPage();  
    do {
      draw();
    } while( u8g.nextPage() );
  }

  change_temp();
  
  // Sleep after turning off screen
  if (!sleeping && (idlebuttons[I_BCOIL] > SLEEP_TIME))
    prepare_sleep();

  // Check if should wake up (MINUS button pressed few times)
  if (sleeping)
    check_unsleep();
  
  unignore_buttons();

  loopcycle++;

//  Serial.println(vmainv);
  if (sleeping && !lock_sleep)
    entering_sleep();
  
  if (((rcoil_zero < 0.01) && (rcoil > 0.01)) || (showtemp < -50)) {
    rcoil_zero = rcoil;
//    update_eeprom = true;
  }

  update_eeprom();

  delay(333);
} // loop
