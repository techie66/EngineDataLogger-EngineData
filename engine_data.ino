#include <SleepyPi2.h>
#include <Time.h>
#include <LowPower.h>
#include <PCF8523.h>
#include <Wire.h>
#include <PinChangeInterrupt.h>

#define SLAVE_ADDRESS 0x04

const unsigned long     MAX_UINT16_T = 4294967295,
                        MICROS_SECOND = 1000000,
                        SHUTDOWN_DELAY = 60000;

const int               bikeOnPin = 12,
			rpmPulsePin = 2,
			wheelPulsePin = 3,
			ledPin = 13;
const uint8_t		rpm_pulses = 10,
      			speed_pulses = 10;
			// MPH_CONV = in/mi * h/min * in/pulse
//speed = ((100 * 60 * MICROS_SECOND / SPEED_interval) * MPH_CONV * speed_pulses);
//const float		MPH_CONV = 1.96071428571*60/63360;
const uint32_t		MPH_CONV = 1.96071428571*60*100*60*MICROS_SECOND*speed_pulses/63360;
//rpm = ((60 * MICROS_SECOND / RPM_interval) * rpm_pulses);
const uint32_t		RPM_CONV = 60*MICROS_SECOND*rpm_pulses;

uint16_t                rpm = 0,
			temp_oil = 0,
			speed = 0;
float			supply_voltage = 0.0;
volatile unsigned long  RPM_interval = MAX_UINT16_T,
	 		SPEED_interval = MAX_UINT16_T,
                        rpm_time_last = 0,
                        speed_time_last = 0,
                        rpm_wkg_micros = 0,
                        speed_wkg_micros = 0;

volatile bool           bike_running = false;
unsigned long           wkgTime = 0,
                        time,
                        time_BikeOff = 0;

volatile uint8_t        rpm_pulse_count = 0,
			speed_pulse_count = 0;;

boolean                 debug = true;

// callback for received data
void receiveData(int byteCount){
  // Maybe store a cmd to output requested data
}

// callback for sending data
void sendData(){
  Wire.write((const uint8_t*)&rpm,sizeof(rpm));
  Wire.write((const uint8_t*)&temp_oil,sizeof(temp_oil));
  Wire.write((const uint8_t*)&speed,sizeof(speed));
  Wire.write((const uint8_t*)&supply_voltage,sizeof(supply_voltage));
}

// ISR to get pulse interval for RPM(engine)
void rpmPulse() {
  rpm_wkg_micros = micros();
  rpm_pulse_count++;
  if (rpm_pulse_count >= rpm_pulses) { 
    RPM_interval = (rpm_wkg_micros - rpm_time_last);
    rpm_time_last = rpm_wkg_micros;
    rpm_pulse_count = 0;
  }
}

// ISR to get pulse interval for wheel(engine)
void wheelPulse() {
  speed_wkg_micros = micros();
  speed_pulse_count++;
  if (speed_pulse_count >= speed_pulses) {
    SPEED_interval = (speed_wkg_micros - speed_time_last);
    speed_time_last = speed_wkg_micros;
    speed_pulse_count = 0;
  }
}

// ISR to signal bike is on
void bikeOn() {
  bike_running = !digitalRead(bikeOnPin);
}

// Output serial debug if debug is true
void debug_out(const char str[]) {
  if (debug) {
    Serial.write(str,strlen(str));
  }
}

void setup() {
  SleepyPi.enableWakeupAlarm(false);
  SleepyPi.rtcInit(false);
  SleepyPi.enablePiPower(false);
  SleepyPi.enableExtPower(false);

  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  // Setup pins
  pinMode(ledPin,OUTPUT); // Interrupt
  pinMode(rpmPulsePin,INPUT_PULLUP); // Interrupt
  pinMode(wheelPulsePin,INPUT_PULLUP); // Interrupt
  pinMode(bikeOnPin, INPUT_PULLUP);

  digitalWrite(ledPin,HIGH);

  // PinChangeInterrupt can always be abbreviated with PCINT
  attachPCINT(digitalPinToPCINT(bikeOnPin), bikeOn, CHANGE);
  disablePCINT(digitalPinToPCINT(bikeOnPin));

  attachInterrupt(digitalPinToInterrupt(rpmPulsePin), rpmPulse, FALLING);//Initialize the intterrupt pin (Arduino digital pin 2);
  attachInterrupt(digitalPinToInterrupt(wheelPulsePin), wheelPulse, FALLING);//Initialize the intterrupt pin (Arduino digital pin 3);
  
  if (debug) {
    Serial.begin(9600); // start serial for output
    Serial.println("Ready!");
  }
}

void loop() {
  bool	pi_running;
  float	rpi_current = 0.0;

  pi_running = SleepyPi.checkPiStatus(50,false);
  bike_running = !digitalRead(bikeOnPin);
  rpi_current = SleepyPi.rpiCurrent();

  if (debug) {
    Serial.print("Current: ");
    Serial.print(rpi_current);
    Serial.print(" pi_running=");
    Serial.print(pi_running);
    Serial.print(" bike_on=");
    Serial.println(bike_running);
  }

  while(pi_running == false && bike_running == false) {
    debug_out("Going to Sleep\r\n");
    delay(500);
    SleepyPi.enablePiPower(false);
    SleepyPi.enableExtPower(false);
    digitalWrite(ledPin,LOW);
    enablePCINT(digitalPinToPCINT(bikeOnPin));
    SleepyPi.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    disablePCINT(digitalPinToPCINT(bikeOnPin));
    digitalWrite(ledPin,HIGH);
    pi_running = SleepyPi.checkPiStatus(50,false);
    bike_running = !digitalRead(bikeOnPin);
  }


  if (bike_running) {
    time_BikeOff = 0;
    if (pi_running) {
      // Turn on power
      // Usually redundant, but keeps from resetting the Pi when turning off software power jumper
      SleepyPi.enablePiPower(true);
      SleepyPi.enableExtPower(true);
    }
    else {
      // Bike is on, but Pi is not, reset power to Pi
      // This handles a situation where Pi shutdown on its own
      SleepyPi.enablePiPower(true);
      SleepyPi.enableExtPower(true);
      debug_out("Wait 5s for pi to start running.\r\n");
      delay(5000);
      pi_running = SleepyPi.checkPiStatus(50,false);
      if (!pi_running) {
        debug_out("Turning off Pi.\r\n");
        SleepyPi.enablePiPower(false);
        SleepyPi.enableExtPower(false);
        delay(50);
        debug_out("Turning on Pi.\r\n");
        SleepyPi.enablePiPower(true);
        SleepyPi.enableExtPower(true);
      }
    }
      
  }
  else {
    if (time_BikeOff == 0) {
      time_BikeOff = time;
    }
    if(pi_running) {
      if (time - time_BikeOff > SHUTDOWN_DELAY) {
        SleepyPi.piShutdown();
        SleepyPi.enableExtPower(false);
        SleepyPi.enablePiPower(false);
      }
    }
  }
  time = millis();

  delay(10);  // voltage reading is artificially high if we don't delay first
  supply_voltage = SleepyPi.supplyVoltage();

  // Calculate RPM
  wkgTime = RPM_interval;
  if (micros() > ( rpm_time_last + MICROS_SECOND ) ){
    wkgTime = MAX_UINT16_T;
    rpm = 0;
  }
  
  if (wkgTime < MAX_UINT16_T) {
    // ( s/min * us/s / us/C*pulse ) * C = RPM
    // RPM_CONV is constant to solve equation since only variable is pulse time
    rpm = (RPM_CONV / wkgTime);
  }

  // Calculate Speed (x100)
  wkgTime = SPEED_interval;
  if (micros() > ( speed_time_last + MICROS_SECOND ) ){
    wkgTime = MAX_UINT16_T;
    speed = 0;
  }
  
  if (wkgTime < MAX_UINT16_T) { 
    // Each pulse is 1.96071428571", SPEED_interval is the time it takes for speed_pules
    // ( A * s/min * us/s) / us/C*in ) * CONV * C = MPH*100
    // A = multiply constant to give us MPH*100
    // CONV = in/mi * h/min * in/pulse
    // C = pulses
    // MPH_CONV is a constant calculated to solve to equation, since the only variable is pulse time
    speed = ( MPH_CONV / wkgTime);
  }
  if (debug) {
      delay(500);
      Serial.print("Speed Pulses:");
      Serial.print(speed_pulse_count,DEC);
      Serial.print(" SPEED: ");
      Serial.println(speed,DEC); 
      Serial.print("RPM Pulses:");
      Serial.print(rpm_pulse_count,DEC);
      Serial.print(" RPM: ");
      Serial.println(rpm,DEC); 
  }
  
}
