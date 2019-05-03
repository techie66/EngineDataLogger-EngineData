#include <EEPROM.h>
#include "../EDL_eeprom.h"

void setup() {
	config mConfig;
	EEPROM.get(0,mConfig);
	//mConfig.odo_address = 128;
	//mConfig.rear_teeth = 42;
	mConfig.rear_diameter = 2500; //2621
	//mConfig.shutdown_delay = 300000;
	EEPROM.put(0,mConfig);

	odometer mOdo;
	EEPROM.get(mConfig.odo_address,mOdo);
        //mOdo.miles_hundredths = 5924201;
        mOdo.trip_hundredths = 5911358;
	EEPROM.put(mConfig.odo_address,mOdo);
}

void loop() {
	// Empty Loop
}
