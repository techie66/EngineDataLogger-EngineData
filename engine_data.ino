#include <SleepyPi2.h>
#include <Time.h>
#include <LowPower.h>
#include <PCF8523.h>
#include <Wire.h>
#include <PinChangeInterrupt.h>
#include <EEPROM.h>
#include "EDL_eeprom.h"

#define 		SLAVE_ADDRESS 0x04

#define			MAX_UINT16_T 4294967295
#define			INCHES_PER_MILE 63360
#define			MICROS_SECOND 1000000
#define			MICROSECONDS_MINUTE MICROS_SECOND * 60
#define			MICROSECONDS_HOUR MICROSECONDS_MINUTE * 60
#define			EEPROM_WRITE_LIMT 50000
#define 		EEPROM_SIZE 1024
#define			EEPROM_CONFIG_BUFFER 128

const int               bikeOnPin = 12,
			rpmPulsePin = 2,
			wheelPulsePin = 3,
			ledPin = 13,
			oil_Temp_pin = 14;

const uint8_t		rpm_pulses = 10,	// How many pulses to average
      			speed_pulses = 200;

volatile uint32_t	running_odometer,
	 		running_trip,
			pulses_per_hundredth_mile,
			odometer_pulses = 0;
float			inches_per_pulse;
unsigned long		SHUTDOWN_DELAY = 60000;
uint32_t		MPH_CONV;
uint32_t		RPM_CONV;

uint16_t                rpm = 0,
			temp_oil = 0,
			speed = 0;
float			supply_voltage = 0.0;
volatile unsigned long  RPM_interval = 0,
	 		SPEED_interval = 0,
                        rpm_time_last = 0,
                        speed_time_last = 0,
                        rpm_wkg_micros = 0,
                        speed_wkg_micros = 0;
volatile unsigned long	rpm_pulse_times[rpm_pulses] = {0},
	 		speed_pulse_times[speed_pulses] = {0};
volatile bool           bike_running = false;
unsigned long           wkgTime = 0,
                        time,
                        time_BikeOff = 0;

volatile uint8_t        rpm_pulse_count = 0,
			speed_pulse_count = 0;;

boolean                 debug = false;

// ISR to get pulse interval for RPM(engine)
// a little longer than I'd like, since it handles the averaging
// but it saves clock cycles overall by only calculating interval once
void rpmPulse() {
  // Save current micros
  rpm_wkg_micros = micros();
  // increment counter and reset if necessary
  rpm_pulse_count++;
  if (rpm_pulse_count >= rpm_pulses) { 
    rpm_pulse_count = 0;
  }
  // decrement RPM_interval by oldest interval
  RPM_interval -= rpm_pulse_times[rpm_pulse_count];
  // save new interval and add to RPM_interval
  rpm_pulse_times[rpm_pulse_count] = (rpm_wkg_micros - rpm_time_last);
  RPM_interval += rpm_pulse_times[rpm_pulse_count];
  // Save last time we were called
  rpm_time_last = rpm_wkg_micros;
}


// ISR to get pulse interval for wheel(engine)
// same documentation as rpmPulse
void wheelPulse() {
  speed_wkg_micros = micros();
  speed_pulse_count++;
  odometer_pulses++;
  if (speed_pulse_count >= speed_pulses) {
    speed_pulse_count = 0;

    /*// non-running average
    SPEED_interval = speed_wkg_micros - speed_time_last;
    speed_time_last = speed_wkg_micros;
    */
  }
  if (odometer_pulses > pulses_per_hundredth_mile) {
	  odometer_pulses = 0;
	  running_odometer++;
  }
  
  SPEED_interval -= speed_pulse_times[speed_pulse_count];
  speed_pulse_times[speed_pulse_count] = (speed_wkg_micros - speed_time_last);
  SPEED_interval += speed_pulse_times[speed_pulse_count];
  speed_time_last = speed_wkg_micros;
  
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

double Thermistor(int RawADC) {
  /*Calculates temperature from analog reading
   * Schematic :
   * +Vs---R1---+---+---R3--GND
   *            |   |
   *           ADC  +--Rth--GND
   *
   * R1 and R3 chosen to provide no more that Vref at ADC 
   * when Thermistor is disconnected (ie Rth=infinity)
   * and acceptable current when thermistor is shorted (ie Rth=0)
   */
  double Temp;
  double Rth; // Thermistor Resistance
  double Vo;
  double R2;
  // Easiest way to calibrate is to adjust the resistor values slightly
  static const double R1 = 460.0; // +side resistor
  static const double R3 = 1000.0; // -side resistor
  static const double Rconn = 0.0; // sensor connection resistance
  static const double Vs = 5.06; // voltage connected to +side resistor
  static const double Vref = 3.26; // analog reference voltage
  static const double SHA = .001600646526;
  static const double SHB = .0002607956449;
  static const double SHC = -0.00000004456663162;

  /*calculations*/
  Vo = (RawADC/1024.0)*Vref;
  R2 = (Vo*R1)/(Vs-Vo);
  Rth = (R3*R2)/(R3-R2)-Rconn;
  Temp = log(Rth); //Temporarily holds the value of log(Rth) to be used below

  // Steinhart-Hart (factored to reduce calculations a bit)
  Temp = 1 / (SHA + (SHB + (SHC * Temp * Temp ))* Temp );
  Temp = Temp - 273.15;            // Convert Kelvin to Celcius
  Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  return Temp;
}

void readEEPROM() {
	int eeAddress = 0;
	config eeprom_config;

	EEPROM.get(eeAddress,eeprom_config);
	odometer saved_odometer;

	EEPROM.get(eeprom_config.odo_address,saved_odometer);
	running_odometer = saved_odometer.miles_hundredths;
	running_trip = saved_odometer.trip_hundredths;

	inches_per_pulse = (eeprom_config.rear_diameter/100.0 * PI) / eeprom_config.rear_teeth;
	pulses_per_hundredth_mile = (INCHES_PER_MILE / 100) / inches_per_pulse;
	
	SHUTDOWN_DELAY = eeprom_config.shutdown_delay;
	MPH_CONV = 100*inches_per_pulse*MICROSECONDS_HOUR*speed_pulses/INCHES_PER_MILE;
	RPM_CONV = MICROSECONDS_MINUTE*rpm_pulses;
}

void saveOdometer() {
	int eeAddress = 0;
	config eeConfig;
	EEPROM.get(eeAddress,eeConfig);

	odometer eeOdometer;
	EEPROM.get(eeConfig.odo_address,eeOdometer);
	if ((running_odometer > eeOdometer.miles_hundredths) || (running_trip > eeOdometer.trip_hundredths)) {
		//Update EEPROM Odometer
		eeOdometer.miles_hundredths = running_odometer;
		eeOdometer.trip_hundredths = running_trip;
		eeOdometer.count++;
		if (eeOdometer.count > EEPROM_WRITE_LIMT) {
			//Rotate address for Odometer
			eeOdometer.count = 0;
			eeConfig.odo_address += sizeof(eeOdometer);
			if (eeConfig.odo_address > EEPROM_SIZE) {
				eeConfig.odo_address = EEPROM_CONFIG_BUFFER;
			}
			EEPROM.put(eeAddress,eeConfig);
		}
		EEPROM.put(eeConfig.odo_address,eeOdometer);
	}
}

// callback for received data
void receiveData(int byteCount){
  // Maybe store a cmd to output requested data
  char cmd;
  while (Wire.available()) {
    cmd = Wire.read();
    if (cmd == 'T') {
      uint8_t s = SREG;
      cli();
      // Do Stuff
      running_trip = running_odometer;
      saveOdometer();
      SREG = s;
    }
  }
}

// callback for sending data
void sendData(){
  Wire.write((const uint8_t*)&rpm,sizeof(rpm));
  Wire.write((const uint8_t*)&temp_oil,sizeof(temp_oil));
  Wire.write((const uint8_t*)&speed,sizeof(speed));
  Wire.write((const uint8_t*)&supply_voltage,sizeof(supply_voltage));
  Wire.write((const uint8_t*)&running_odometer,sizeof(running_odometer));
  Wire.write((const uint8_t*)&running_trip,sizeof(running_trip));
}

void setup() {
  // Setup "constants" configurable
  readEEPROM();

  SleepyPi.enableWakeupAlarm(false); // 2019-04-25 suddenly made the sleepypi stop working
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
  static long int oil_Temp_rb[10] = {0};
  static long int oil_Temp_sum = 0;
  static int oil_Temp_rb_i = 0;

  bool pi_running = SleepyPi.checkPiStatus(50,false);
  bike_running = !digitalRead(bikeOnPin);
  float rpi_current = SleepyPi.rpiCurrent();

  if (debug) {
    Serial.print("Current: ");
    Serial.print(rpi_current);
    Serial.print(" pi_running=");
    Serial.print(pi_running);
    Serial.print(" bike_on=");
    Serial.println(bike_running);
  }

  if (debug) {
    bike_running = true;
    pi_running = true;
  }

  while(pi_running == false && bike_running == false) {
    debug_out("Going to Sleep\r\n");
    saveOdometer();
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
  cli();
  wkgTime = RPM_interval;
  sei();
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
  cli();
  wkgTime = SPEED_interval;
  sei();
  if (micros() > ( speed_time_last + MICROS_SECOND ) ){
    wkgTime = MAX_UINT16_T;
    speed = 0;
  }
  
  if (wkgTime < MAX_UINT16_T) { 
    // MPH_CONV is a constant calculated to solve to equation, since the only variable is pulse time
    speed = ( MPH_CONV / wkgTime);
  }

  // Keep track of Oil Temp
  oil_Temp_rb_i++;
  if (oil_Temp_rb_i >= 10) {
    oil_Temp_rb_i=0;
  }
  oil_Temp_sum -= oil_Temp_rb[oil_Temp_rb_i];
  oil_Temp_rb[oil_Temp_rb_i] = Thermistor(analogRead(oil_Temp_pin)) * 100;
  oil_Temp_sum += oil_Temp_rb[oil_Temp_rb_i];
  temp_oil = oil_Temp_sum / 10;

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
