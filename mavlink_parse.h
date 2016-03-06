#pragma once
#define ToDeg(x) (x*57.2957795131)  

#include "telemetry.h"

#ifdef ARDUCOPTER
//http://copter.ardupilot.com/wiki/arducopter-parameters/#flight_mode_1_arducopterfltmode1

#define MODE_STABILIZE 	0
#define MODE_ACRO		1
#define MODE_ALTHOLD 	2
#define MODE_AUTO 		3
#define MODE_GUIDED		4
#define MODE_LOITER		5
#define MODE_RTL 		6
#define MODE_CIRCLE		7
#define MODE_LAND		9
#define MODE_DRIFT		11
#define MODE_SPORT		13
#define MODE_FLIP		14
#define MODE_AUTOTUNE 	15
#define MODE_POSHOLD 	16
#define MODE_BRAKE		17
#define MODE_THROW		18

#endif

#ifdef ARDUPLANE
//http://plane.ardupilot.com/wiki/arduplane-parameters/#flightmode1_arduplanefltmode1

#define MODE_MANUAL 	0
#define MODE_CIRCLE		1
#define MODE_STABILIZE 	2
#define MODE_TRAINING	3
#define MODE_ACRO		4
#define MODE_FBWA		5
#define MODE_FBWB 		6
#define MODE_CRUISE		7
#define MODE_AUTOTUNE	8
#define MODE_AUTO		10
#define MODE_RTL		11
#define MODE_LOITER		12
#define MODE_GUIDED 	15

#endif


int mavlink_parse_buffer(telemetry_data_t *td, uint8_t *buf, int buflen);



