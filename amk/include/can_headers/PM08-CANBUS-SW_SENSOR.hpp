//Generated on Wed Jun 15 10:27:29 2022
#ifndef SWSENSOR
#define SWSENSOR

#include <cstdint>

namespace PUTM_CAN {

enum struct SWPS_states: uint8_t {
	Transmitting,
};

struct __attribute__ ((packed)) SWPS_main{
	uint16_t steeringAngle;
};


const uint16_t SWPS_MAIN_CAN_ID = 0x9b;
const uint8_t SWPS_MAIN_CAN_DLC = sizeof(SWPS_main);
const uint8_t SWPS_MAIN_FREQUENCY = 1;

} // namespace can

#endif

