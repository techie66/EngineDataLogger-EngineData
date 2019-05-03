#ifndef EDL_EEPROM_H
#define EDL_EEPROM_H
struct	config {
	uint16_t	odo_address;
	uint8_t		rear_teeth;
	uint16_t	rear_diameter; // inches x 100. eg 8269 for 82.69"
	unsigned long	shutdown_delay = 60000;
};

struct odometer {
	uint32_t	miles_hundredths;
	uint16_t	count;
	uint32_t	trip_hundredths;
};
#endif
