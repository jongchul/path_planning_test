//threshold for lane change : 
// minimum distance between ego car and other cars on the road.

/* VEHICLE KINETIC CONSTANT */

#define FOLLOW_DISTANCE 20  

#define DISTANCE_BETWEEN_CARS 30.0  

//#define MAX_INSTANTANEOUS_ACCEL 5
#define MAX_INSTANTANEOUS_ACCEL 5

//MAX S
#define MAX_S 6945.554



/* VEHICLE TRAJECTORY CONSTANT */

//other car's trajectory 
//50 points during the duration. 
#define N_SAMPLES 50

// m/s
#define SPEED_LIMIT 22.35
//#define SPEED_LIMIT 20.0


// second
#define DURATION 1

// splie patch
#define SPLINE_PATCH 2



/* COST FUNCTION CONSTANT */

// gole lane, middle lane in the right side.
#define GOLE_LANE 1

//S value for end of the track.
#define GOLE_S 6914.14925765991

//maximum vehicle sensing distance .
#define MAX_SENSING_DIST 500


//trajectory generation number .
#define TR_NUM 10

#define SIGMA_S = [10.0, 4.0, 2.0] 



