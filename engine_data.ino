#include <SleepyPi2.h>
#include <Time.h>
#include <LowPower.h>
#include <PCF8523.h>
#include <Wire.h>
#include <PinChangeInterrupt.h>
#include <EEPROM.h>
#include "EDL_eeprom.h"
#include <mcp2515_can.h>

#define     SLAVE_ADDRESS 0x04
// CAN Constants
#define SPI_CS_PIN 10
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin


#define     MAX_UINT16_T 4294967295
#define     INCHES_PER_MILE 63360
#define     MICROS_SECOND 1000000
#define     MICROSECONDS_MINUTE MICROS_SECOND * 60
#define     MICROSECONDS_HOUR MICROSECONDS_MINUTE * 60
#define     EEPROM_WRITE_LIMT 50000
#define     EEPROM_SIZE 1024
#define     EEPROM_CONFIG_BUFFER 128
#define     CANID_RESET_TRIP 0x230
#define     CANID_ENGINE1 0x227
#define     CANID_ENGINE2 0x228
#define     CAN_ENGINE1_RATE 200
#define     CAN_ENGINE2_RATE 5000

const int   bikeOnPin = 9,
            rpmPulsePin = 2,
            wheelPulsePin = 3,
            oil_Temp_pin = A0,
            oil_pres_pin = A1;

const uint8_t   rpm_pulses = 10,  // How many pulses to average
                speed_pulses = 200;

volatile uint32_t running_odometer,
         running_trip,
         pulses_per_hundredth_mile,
         odometer_pulses = 0;
volatile bool ready_to_save_odo = false;
const int odo_save_interval_hundredths = 100;
float inches_per_pulse;
unsigned long SHUTDOWN_DELAY = 60000;
uint32_t MPH_CONV;
uint32_t RPM_CONV;

uint16_t rpm = 0,
         temp_oil = 0, // hundredths, F
         speed = 0,
         pres_oil = 0; // hundredths, psi
float supply_voltage = 0.0;
volatile unsigned long  RPM_interval = 0,
                        SPEED_interval = 0,
                        rpm_time_last = 0,
                        speed_time_last = 0,
                        rpm_wkg_micros = 0,
                        speed_wkg_micros = 0;
volatile unsigned long  rpm_pulse_times[rpm_pulses] = {0},
    speed_pulse_times[speed_pulses] = {0};
volatile bool bike_running = false;
unsigned long wkgTime = 0,
              time,
              time_BikeOff = 0;

volatile uint8_t rpm_pulse_count = 0,
                 speed_pulse_count = 0;;

boolean debug = false;

// ISR to get pulse interval for RPM(engine)
// a little longer than I'd like, since it handles the averaging
// but it saves clock cycles overall by only calculating interval once
void rpmPulse()
{
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
void wheelPulse()
{
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
    static int running_ticks_to_save = 0;
    odometer_pulses = 0;
    running_odometer++;
    running_ticks_to_save++;
    if ( running_ticks_to_save >= odo_save_interval_hundredths ) {
      ready_to_save_odo = true;
      running_ticks_to_save = 0;
    }
  }

  SPEED_interval -= speed_pulse_times[speed_pulse_count];
  speed_pulse_times[speed_pulse_count] = (speed_wkg_micros - speed_time_last);
  SPEED_interval += speed_pulse_times[speed_pulse_count];
  speed_time_last = speed_wkg_micros;

}

// ISR to signal bike is on
void bikeOn()
{
  bike_running = !digitalRead(bikeOnPin);
}

// Output serial debug if debug is true
void debug_out(const char str[])
{
  if (debug) {
    Serial.write(str, strlen(str));
  }
}

double Thermistor(int RawADC)
{
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
  Vo = (RawADC / 1024.0) * Vref;
  R2 = (Vo * R1) / (Vs - Vo);
  Rth = (R3 * R2) / (R3 - R2) - Rconn;
  Temp = log(Rth); //Temporarily holds the value of log(Rth) to be used below

  // Steinhart-Hart (factored to reduce calculations a bit)
  Temp = 1 / (SHA + (SHB + (SHC * Temp * Temp )) * Temp );
  Temp = Temp - 273.15;            // Convert Kelvin to Celcius
  Temp = (Temp * 9.0) / 5.0 + 32.0; // Convert Celcius to Fahrenheit
  return Temp;
}

double oilPressure( int RawADC ) {
	/* oilPressure
	   * Calculate oil pressure from ADC reading
	   *
	   *
	*/
	static const double Vref = 3.26;
	// Generic conversion factor from V to pressure unit, assumed linear
  /*  Ambient = 14.7 psi = 0.5V from transducer = 0.25V after voltage divider
   *  Max = 150 psi = 4.5V from transducer = 2.25V after voltage divider
   *  Calculate slope of regression line and that is Vconv
  */
	static const double Vconv = 75.0; // Max = 150
  static const double pfac = 0; // linear correction
  //static const double pfac = 18.75; // linear correction
	//static const double Vconv = 792.6; // Max = 1600
  //static const double pfac = 175; // linear correction
  double intermediate = (((RawADC/1024.0) * Vref)) * Vconv;
  if ( intermediate > pfac ) return intermediate - pfac; 
  else return 0;
}

/*
 * Basic linear compensation for presure reading
 * not very accurate
 */
uint16_t compensateOilPressure(uint16_t p, uint16_t t)
{
  uint16_t compensatedPressure=p;
  double compensationFactor = 0.037222;
  if ( t > 7000) {
    compensatedPressure = p + ((t - 7000) * compensationFactor);
  }
  return compensatedPressure;
}

void readEEPROM()
{
  int eeAddress = 0;
  config eeprom_config;

  EEPROM.get(eeAddress, eeprom_config);
  odometer saved_odometer;

  EEPROM.get(eeprom_config.odo_address, saved_odometer);
  running_odometer = saved_odometer.miles_hundredths;
  running_trip = saved_odometer.trip_hundredths;

  inches_per_pulse = (eeprom_config.rear_diameter / 100.0 * PI) / eeprom_config.rear_teeth;
  pulses_per_hundredth_mile = (INCHES_PER_MILE / 100) / inches_per_pulse;

  SHUTDOWN_DELAY = eeprom_config.shutdown_delay;
  MPH_CONV = 100 * inches_per_pulse * MICROSECONDS_HOUR * speed_pulses / INCHES_PER_MILE;
  RPM_CONV = MICROSECONDS_MINUTE * rpm_pulses;
}

void saveOdometer()
{
  int eeAddress = 0;
  config eeConfig;
  EEPROM.get(eeAddress, eeConfig);

  odometer eeOdometer;
  EEPROM.get(eeConfig.odo_address, eeOdometer);
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
      EEPROM.put(eeAddress, eeConfig);
    }
    EEPROM.put(eeConfig.odo_address, eeOdometer);
  }
}

// callback for received data
void receiveData(int byteCount)
{
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
void sendData()
{
  Wire.write((const uint8_t *)&rpm, sizeof(rpm));
  Wire.write((const uint8_t *)&temp_oil, sizeof(temp_oil));
  Wire.write((const uint8_t *)&speed, sizeof(speed));
  Wire.write((const uint8_t *)&supply_voltage, sizeof(supply_voltage));
  Wire.write((const uint8_t *)&running_odometer, sizeof(running_odometer));
  Wire.write((const uint8_t *)&running_trip, sizeof(running_trip));
  Wire.write((const uint8_t *)&pres_oil, sizeof(pres_oil));
}

void setup()
{
  if (debug) {
    Serial.begin(9600); // start serial for output
    Serial.println("Ready!");
  }
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
  pinMode(rpmPulsePin, INPUT_PULLUP); // Interrupt
  pinMode(wheelPulsePin, INPUT_PULLUP); // Interrupt
  pinMode(bikeOnPin, INPUT_PULLUP);
  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_12MHz)) {            // init can bus : baudrate = 500k
    delay(100);
    if (debug) {
      Serial.println("CAN Initialization failing!");
    }
  }

  CAN.mcpPinMode(MCP_RX0BF,MCP_PIN_OUT);
  CAN.mcpDigitalWrite(MCP_RX0BF,LOW);

  // PinChangeInterrupt can always be abbreviated with PCINT
  attachPCINT(digitalPinToPCINT(bikeOnPin), bikeOn, CHANGE);
  disablePCINT(digitalPinToPCINT(bikeOnPin));

  attachInterrupt(digitalPinToInterrupt(rpmPulsePin), rpmPulse, FALLING);//Initialize the intterrupt pin (Arduino digital pin 2);
  attachInterrupt(digitalPinToInterrupt(wheelPulsePin), wheelPulse, FALLING);//Initialize the intterrupt pin (Arduino digital pin 3);

}

void loop()
{
  static long int oil_Temp_rb[10] = {0};
  static long int oil_Temp_sum = 0;
  static int oil_Temp_rb_i = 0;

  bool pi_running = SleepyPi.checkPiStatus(50, false);
  bike_running = !digitalRead(bikeOnPin);
  float rpi_current = SleepyPi.rpiCurrent();

  if ( ready_to_save_odo ) {
    saveOdometer();
    ready_to_save_odo = false;
  }

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

  while (pi_running == false && bike_running == false) {
    debug_out("Going to Sleep\r\n");
    saveOdometer();
    delay(500);
    SleepyPi.enablePiPower(false);
    SleepyPi.enableExtPower(false);
    enablePCINT(digitalPinToPCINT(bikeOnPin));
    CAN.mcpDigitalWrite(MCP_RX0BF,HIGH);
    SleepyPi.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    disablePCINT(digitalPinToPCINT(bikeOnPin));
    CAN.mcpDigitalWrite(MCP_RX0BF,LOW);
    pi_running = SleepyPi.checkPiStatus(50, false);
    bike_running = !digitalRead(bikeOnPin);
  }


  if (bike_running) {
    time_BikeOff = 0;
    if (pi_running) {
      // Turn on power
      // Usually redundant, but keeps from resetting the Pi when turning off software power jumper
      SleepyPi.enablePiPower(true);
      SleepyPi.enableExtPower(true);
    } else {
      // Bike is on, but Pi is not, reset power to Pi
      // This handles a situation where Pi shutdown on its own
      SleepyPi.enablePiPower(true);
      SleepyPi.enableExtPower(true);
      debug_out("Wait 5s for pi to start running.\r\n");
      delay(5000);
      pi_running = SleepyPi.checkPiStatus(50, false);
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

  } else {
    if (time_BikeOff == 0) {
      time_BikeOff = time;
    }
    if (pi_running) {
      if (time - time_BikeOff > SHUTDOWN_DELAY) {
        SleepyPi.piShutdown(50); // argument is threshold mA to detect pi is shut off
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
  if (micros() > ( rpm_time_last + MICROS_SECOND ) ) {
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
  if (micros() > ( speed_time_last + MICROS_SECOND ) ) {
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
    oil_Temp_rb_i = 0;
  }
  oil_Temp_sum -= oil_Temp_rb[oil_Temp_rb_i];
  oil_Temp_rb[oil_Temp_rb_i] = Thermistor(analogRead(oil_Temp_pin)) * 100;
  oil_Temp_sum += oil_Temp_rb[oil_Temp_rb_i];
  temp_oil = oil_Temp_sum / 10;


  // Store oil pressure
  delay(10); // allow ADC to settle
  pres_oil = oilPressure(analogRead(oil_pres_pin)) * 100;
  pres_oil = compensateOilPressure(pres_oil,temp_oil);

  if (debug) {
    delay(500);
    Serial.print("Speed Pulses:");
    Serial.print(speed_pulse_count, DEC);
    Serial.print(" SPEED: ");
    Serial.println(speed, DEC);
    Serial.print("RPM Pulses:");
    Serial.print(rpm_pulse_count, DEC);
    Serial.print(" RPM: ");
    Serial.println(rpm, DEC);
  }

  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    static unsigned long CanId = 0;
    byte CanLen;
    byte CanBuf[8];
    byte canReadRet = CAN.readMsgBuf(&CanLen, (byte *)&CanBuf);
    CanId = CAN.getCanId();
    if ( CanId == CANID_RESET_TRIP  && CanBuf[0] == 0xFF) {
      uint8_t s = SREG;
      cli();
      // Do Stuff
      running_trip = running_odometer;
      saveOdometer();
      SREG = s;
    }
  }
  static unsigned long last_engine1 = 0;
  if ( last_engine1 + CAN_ENGINE1_RATE < millis()) {
    byte _response[8] = {0};
    _response[0] = rpm;
    _response[1] = rpm >> 8;
    uint8_t _speed = speed / 100;
    _response[2] = _speed;
    uint8_t _pres = pres_oil / 100;
    _response[3] = _pres;
    uint16_t _temp = (temp_oil / 100) + 50;
    _response[4] = _temp;
    _response[5] = _temp >> 8;
    uint8_t _volts = supply_voltage * 10.0;
    _response[6] = _volts;
    CAN.sendMsgBuf(CANID_ENGINE1, 0, 7, _response);
    last_engine1 = millis();
  }
  static unsigned long last_engine2 = 0;
  if ( last_engine2 + CAN_ENGINE2_RATE < millis()) {
    byte _response[8] = {0};
    uint32_t _trip = (running_odometer - running_trip) * 1.609344 / 10.0;
    uint32_t _odo = running_odometer * 1.609344 / 10.0;
    _response[0] = _odo;
    _response[1] = _odo >> 8;
    _response[2] = _odo >> 16;
    _response[3] = _trip;
    _response[4] = _trip >> 8;
    _response[5] = _trip >> 16;
    CAN.sendMsgBuf(CANID_ENGINE2, 0, 6, _response);
    last_engine2 = millis();
  }
}
