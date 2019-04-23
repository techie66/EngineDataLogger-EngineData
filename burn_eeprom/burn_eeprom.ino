#include <EEPROM.h>
struct	mconfig {
	uint16_t	odo_address;
	uint8_t		rear_teeth;
	uint16_t	rear_diameter; // inches x 100. eg 8269 for 82.69"
	unsigned long	shutdown_delay = 60000;
};

struct odometer {
	uint32_t	miles_hundredths;
	uint16_t	count;
};


void setup() {
	mconfig mConfig;
	mConfig.odo_address = 128;
	mConfig.rear_teeth = 42;
	mConfig.rear_diameter = 2621;
	mConfig.shutdown_delay = 300000;

	odometer mOdo = {5910835,0};
	EEPROM.put(0,mConfig);
	EEPROM.put(mConfig.odo_address,mOdo);
}

void loop() {
	// Empty Loop
}
