
#ifndef _SRC_CONSTANTS_H_
 
#define _SRC_CONSTANTS_H_ 

// road lane width in meters
const double LANE_WIDTH=4.0;

// MPH to m/s conversion constans 
const double K_MILE_TO_KM = 1.60934;
const double M_IN_KM = 1000.0;
const double SEC_PER_HOUR = 3600.0;

// sensorf fusion options
const double SENSOR_FUSION_FRONT_BUFFER_K=1.25;
const double SENSOR_FUSION_BACK_BUFFER_K=0.75;

// path generation options
const double PATH_FUTURE_STEP_SIZE=50.0;
const double PATH_POS_AHEAD=51.0;

const unsigned PATH_NUMBER_OF_POINTS = 50U;


// Car velocity settings miles per hour
const double SPEED_LIMIT=49.5;
const double SPEED_INITIAL=0.01;
const double SPEED_DELTA_U=0.224;
const double SPEED_DELTA_D=0.294;

#endif 

