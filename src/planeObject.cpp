/* Implementation of planeObject.h

*/

#include "ros/ros.h"
#include "real/planeObject.h"
#include "real/SimulatedPlane.h"

#include <math.h>
#include "real/standardFuncs.h" /* for PI, EARTH_RADIUS in meters */

/* Implementation of the default constructor: Member variables are set to zero */
real::PlaneObject::PlaneObject(void) {
	this->id = 0;
	this->currentLoc.altitude = 0.0;
	this->currentLoc.latitude = 0.0;
	this->currentLoc.longitude = 0.0;
	this->previousLoc.altitude = 0.0;
	this->previousLoc.latitude = 0.0;
	this->previousLoc.longitude = 0.0;
	this->targetBearing = 0.0;
	this->currentBearing = 0.0;

	this->speed = 0.0;
	this->destination.latitude = 0.0;
	this->destination.longitude = 0.0;
	this->destination.altitude = 0.0;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->collisionRadius = 0.0;

/*	ATTEMPT #1

	//zigzagging
	this->lastGreatestThreat = -1;
	this->lastZEM = -1;
	this->lastZEMTime = -1;
*/

/////////////////////// * ATTEMPT 2 * ///////////////////////////////
	this->twoAgoThreatID = -1;
	this->twoAgoZEM = -1;
	this->twoAgoTgo = -1;

	this->oneAgoThreatID = -1;
	this->oneAgoZEM = -1;
	this->oneAgoTgo = -1;

	this->currentThreatID = -1;
	this->currentZEM = -1;
	this->currentTgo = -1;
	this->antiZigzag = 0;
/////////////////////////////////////////////////////////////////////

}
/* Explicit value constructor using TelemetryUpdate */
real::PlaneObject::PlaneObject(double cRadius, const real::TelemetryUpdate &msg) {
	this->id = msg.planeID;
	this->currentLoc.altitude = msg.currentAltitude;
	this->currentLoc.latitude = msg.currentLatitude;
	this->currentLoc.longitude = msg.currentLongitude;
	this->previousLoc.altitude = 0.0;
	this->previousLoc.latitude = 0.0;
	this->previousLoc.longitude = 0.0;
	this->targetBearing = msg.targetBearing;
	this->currentBearing = 0.0;

	this->speed = msg.groundSpeed;
	this->destination.latitude = msg.destLatitude;
	this->destination.longitude = msg.destLongitude;
	this->destination.altitude = msg.destAltitude;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->collisionRadius = cRadius;


/*	ATTEMPT #1
	//zigzagging
	this->lastGreatestThreat = -1;
	this->lastZEM = -1;
	this->lastZEMTime = -1;
*/

/////////////////////// * ATTEMPT 2 * ///////////////////////////////
	this->twoAgoThreatID = -1;
	this->twoAgoZEM = -1;
	this->twoAgoTgo = -1;

	this->oneAgoThreatID = -1;
	this->oneAgoZEM = -1;
	this->oneAgoTgo = -1;

	this->currentThreatID = -1;
	this->currentZEM = -1;
	this->currentTgo = -1;
	this->antiZigzag = 0;
/////////////////////////////////////////////////////////////////////

}

/* mutator functions to update member variables */
void real::PlaneObject::setID(int id){
	this->id = id;
}

void real::PlaneObject::setPreviousLoc(double lat, double lon, double alt) {
	this->previousLoc.latitude = lat;
	this->previousLoc.longitude = lon;
	this->previousLoc.altitude = alt;
}

void real::PlaneObject::setCurrentLoc(double lat, double lon, double alt) {
	this->currentLoc.latitude = lat;
	this->currentLoc.longitude = lon;
	this->currentLoc.altitude = alt;
}

void real::PlaneObject::setTargetBearing(double tBearing) {
	this->targetBearing = tBearing;
}

void real::PlaneObject::setCurrentBearing(double cBearing) {
	this->currentBearing = cBearing;
}

void real::PlaneObject::setSpeed(double speed) {
	this->speed = speed;
}

void real::PlaneObject::setDestination(const real::waypoint &destination) {
	this->destination = destination;
}


/*	ATTEMPT #1
//zigzagging
void real::PlaneObject::setLastGreatestThreat(int lgt) {
	this->lastGreatestThreat = lgt;
}

void real::PlaneObject::setLastZEM(int ZEM) {
	this->lastZEM = ZEM;
}

void real::PlaneObject::setLastZEMTime() {
	this->lastZEMTime = lastUpdateTime;
}
*/

/////////////////////// * ATTEMPT 2 * ///////////////////////////////
void real::PlaneObject::setTwoAgoThreatID(int ID) {
	this->twoAgoThreatID = ID;
}
void real::PlaneObject::setTwoAgoZEM(double ZEM) {
	this->twoAgoZEM = ZEM;
}
void real::PlaneObject::setTwoAgoTgo(double Tgo) {
	this->twoAgoTgo = Tgo;
}

void real::PlaneObject::setOneAgoThreatID(int ID) {
	this->oneAgoThreatID = ID;
}
void real::PlaneObject::setOneAgoZEM(double ZEM) {
	this->oneAgoZEM = ZEM;
}
void real::PlaneObject::setOneAgoTgo(double Tgo) {
	this->oneAgoTgo = Tgo;
}

void real::PlaneObject::setCurrentThreatID(int ID) {
	this->currentThreatID = ID;
}
void real::PlaneObject::setCurrentZEM(double ZEM) {
	this->currentZEM = ZEM;
}
void real::PlaneObject::setCurrentTgo(double Tgo) {
	this->currentTgo = Tgo;
}

void real::PlaneObject::setAntiZigzag(int zig){
	this->antiZigzag = zig;
}
/////////////////////////////////////////////////////////////////////

void real::PlaneObject::updateTime(void) {
	this->lastUpdateTime = ros::Time::now().toSec();
}


void real::PlaneObject::update(const real::TelemetryUpdate &msg) {

	//Calculate actual Cardinal Bearing
	this->setPreviousLoc(this->currentLoc.latitude, this->currentLoc.longitude, this->currentLoc.altitude);
	this->setCurrentLoc(msg.currentLatitude, msg.currentLongitude, msg.currentAltitude);

	double numerator = (this->currentLoc.latitude - this->previousLoc.latitude);
	double denominator = (this->currentLoc.longitude - this->previousLoc.longitude);
	double angle = atan2(numerator*DELTA_LAT_TO_METERS,denominator*DELTA_LON_TO_METERS)*180/PI;

	/*if (this->id == 5 || this->id == 15) {
		ROS_WARN("Num: %f Den: %f Ang: %f", numerator, denominator, angle);
	}*/
	if (this->currentLoc.latitude != this->previousLoc.latitude && this->currentLoc.longitude != this->previousLoc.longitude)
	{ 
			this->setCurrentBearing(toCardinal(angle));
	}
	else this->setCurrentBearing(0.0);

	this->setTargetBearing(msg.targetBearing);

	this->setSpeed(msg.groundSpeed);
	this->updateTime();
}

/* accessor functions */
int real::PlaneObject::getID(void) const {
	return this->id;
}

real::coordinate real::PlaneObject::getPreviousLoc(void) const {
	return this->previousLoc;
}

real::coordinate real::PlaneObject::getCurrentLoc(void) const {
	return this->currentLoc;
}

double real::PlaneObject::getTargetBearing(void) const {
	return this->targetBearing;
}

double real::PlaneObject::getCurrentBearing(void) const {
	return this->currentBearing;
}
	
double real::PlaneObject::getSpeed(void) const {
	return this->speed;
}

double real::PlaneObject::getLastUpdateTime(void) const {
	return this->lastUpdateTime;
}

real::waypoint real::PlaneObject::getDestination(void) const {
	return this->destination;
}


/*	ATTEMPT #1
//zigzagging
int real::PlaneObject::getLastGreatestThreat(void) const {
	return this->lastGreatestThreat;
}

int real::PlaneObject::getLastZEM(void) const {
	return this->lastZEM;
}

double real::PlaneObject::getLastZEMTime(void) const {
	return this->lastZEMTime;
}
*/

/////////////////////// * ATTEMPT 2 * ///////////////////////////////
int real::PlaneObject::getTwoAgoThreatID(void) const {
	return this->twoAgoThreatID;
}
double real::PlaneObject::getTwoAgoZEM(void) const {
	return this->twoAgoZEM;
}
double real::PlaneObject::getTwoAgoTgo(void) const {
	return this->twoAgoTgo;
}

int real::PlaneObject::getOneAgoThreatID(void) const {
	return this->oneAgoThreatID;
}
double real::PlaneObject::getOneAgoZEM(void) const {
	return this->oneAgoZEM;
}
double real::PlaneObject::getOneAgoTgo(void) const {
	return this->oneAgoTgo;
}

int real::PlaneObject::getCurrentThreatID(void) const {
	return this->currentThreatID;
}
double real::PlaneObject::getCurrentZEM(void) const {
	return this->currentZEM;
}
double real::PlaneObject::getCurrentTgo(void) const {
	return this->currentTgo;
}

int real::PlaneObject::getAntiZigzag(void) const {
	return this->antiZigzag;
}
/////////////////////////////////////////////////////////////////////

/* Find distance between this plane and another plane, returns in meters */
double real::PlaneObject::findDistance(const real::PlaneObject& plane) const {
	return this->findDistance(plane.currentLoc.latitude, plane.currentLoc.longitude);
}


/* Find distance between this plane and another pair of coordinates, 
returns value in meters */
double real::PlaneObject::findDistance(double lat2, double lon2) const {
	double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
	double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;

	return sqrt(pow(xdiff, 2) + pow(ydiff, 2));
}

/* Find Cartesian angle between this plane and another plane, using this plane
as the origin */
double real::PlaneObject::findAngle(const real::PlaneObject& plane) const {
	return this->findAngle(plane.currentLoc.latitude, plane.currentLoc.longitude);
}

/* Find Cartesian angle between this plane and another plane's latitude/longitude 
using this plane as the origin */
double real::PlaneObject::findAngle(double lat2, double lon2) const {
	double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
	double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;
	
	return atan2(ydiff, xdiff);
}


bool real::PlaneObject::isBehind(const real::PlaneObject& plane2, bool turnRight) const {

    	double theta = this->findAngle(plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
	double cartBearing = toCartesian(this->currentBearing);
	double cartBearingBar = manipulateAngle(cartBearing - 180.0);

	if (turnRight && theta < cartBearing && theta > cartBearingBar) return true;
	else if (turnRight) return false;
	else if (theta > cartBearing && theta < cartBearingBar) return true;
	else return false;
}


real::PlaneObject& real::PlaneObject::operator=(const real::PlaneObject& plane) {

	this->id = plane.id;
	this->currentLoc.altitude = plane.currentLoc.altitude;
	this->currentLoc.latitude = plane.currentLoc.latitude;
	this->currentLoc.longitude = plane.currentLoc.longitude;

	this->previousLoc.altitude = plane.previousLoc.altitude;
	this->previousLoc.latitude = plane.previousLoc.latitude;
	this->previousLoc.longitude = plane.previousLoc.longitude;

	this->destination.latitude = plane.destination.latitude;
	this->destination.longitude = plane.destination.longitude;
	this->destination.altitude = plane.destination.latitude;

	this->targetBearing = plane.targetBearing;
	this->currentBearing = plane.currentBearing;

	this->speed = plane.speed;
	this->lastUpdateTime = plane.lastUpdateTime;
	this->collisionRadius = plane.collisionRadius;


/////////////////////// * ATTEMPT 2 * ///////////////////////////////
	this->twoAgoThreatID = plane.twoAgoThreatID;
	this->twoAgoZEM = plane.twoAgoZEM;
	this->twoAgoTgo = plane.twoAgoTgo;

	this->oneAgoThreatID = plane.oneAgoThreatID;
	this->oneAgoZEM = plane.oneAgoZEM;
	this->oneAgoTgo = plane.oneAgoTgo;

	this->currentThreatID = plane.currentThreatID;
	this->currentZEM = plane.currentZEM;
	this->currentTgo = plane.currentTgo;
/////////////////////////////////////////////////////////////////////

	return *this;
}
