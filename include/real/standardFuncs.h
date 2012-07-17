/* standardFuncs
 
A small collection of functions related to UAV flight that are used
throughout the code. Note: All angles are in degrees. */

#ifndef STANDARD_FUNCS
#define STANDARD_FUNCS

/* Constants for converting latitude/longitude to meters */
#define DELTA_LAT_TO_METERS 111200
#define DELTA_LON_TO_METERS 93670

#include "real/standardDefs.h" /* for EARTH_RADIUS in meters */
const double PI = 4.0*atan(1.0);
const double DEGREE_TO_RAD = PI/180.0; /* convert degrees to radians */

namespace real{

	struct coordinate {
		double latitude;
		double longitude;
		double altitude;
	};

	struct threatContainer {
		int planeID;
		double ZEM;
		double timeToGo;
	};
	
	struct waypointContainer {
		real::waypoint plane1WP;
		real::waypoint plane2WP;
		int plane2ID;
	};
}


/* Takes the bearing given by the UAV that is based off of Cardinal directions and converts
it to the equivalent bearing in the Cartesian plane.  Returns a value on the interval [-180, 180]. */
double toCartesian(double UAVBearing);

/* Takes an angle based on the Cartestian plane and converts to the equivalent 
Cardinal direction.  Returns a value on the interval [-180, 180]. */
double toCardinal(double angle);

/* Manipulates the angle parameter so it is always on the interval [-180, 180]. */
double manipulateAngle(double angle);

/* Returns the distance between two points of latitude and longitude in meters.  The first two parameters
are the latitude and longitude of the starting point, and the last two parameters are the latitude and
longitude of the ending point. */
double findDistance(double lat1, double long1, double lat2, double long2);

/* Returns the Cartesian angle between two points of latitude and longitude in degrees.  The starting point
is given by lat1 and long1 (the first two parameters), and the final point is given by lat2 and long2 (the
final two parameters). The value returned is on the interval [-180, 180]. */
double findAngle(double lat1, double long1, double lat2, double long2);

/* Returns the sign of the double. */
double findSign(double number);

/* This function takes an angle from -180 to 180 and returns its signed supplement (i.e. -45 returns -135 and 45 returns 135). */
double calculateSupplement(double theta);

#endif