/*
RIPNA.cpp

This is the implementation of RIPNA.h.
*/


#include <math.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <iostream>

#include "real/ripna.h"
#include "real/standardFuncs.h"		// for PI, EARTH_RADIUS, MPS_SPEED
#include "real/SimulatedPlane.h"		// for MAXIMUM_TURNING_ANGLE

#define CHATTERING_ANGLE 30.0 //degrees
#define SECOND_THRESHOLD 1.50*MPS_SPEED //meters
#define PLANE_MAX_TURN_ANGLE 22.5 //degrees / sec
#define CHECK_ZONE 10.0*MPS_SPEED //meters
#define DANGER_ZEM 3.5*MPS_SPEED //meters
#define MINIMUM_TURNING_RADIUS 28.64058013 //meters
#define DESIRED_SEPARATION 2.5*MPS_SPEED //meters
#define LAMBDA 0.1 //dimensionless
#define TIME_STEP 1.0 //seconds
#define MINIMUM_TIME_TO_GO 100.0 //seconds

/* This is the function called by collisionAvoidance.cpp that calls 
all necessary functions in order to generate the new collision avoidance 
waypoint. If no collision avoidance maneuvers are necessary, the function
returns the current destination waypoint. */
real::waypoint real::findNewWaypoint(PlaneObject &plane1, std::map<int, PlaneObject> &planes) {
    
    /* Find plane to avoid*/
	real::threatContainer greatestThreat = findGreatestThreat(plane1, planes);
	
	/* Unpack plane to avoid*/	
	int threatID = greatestThreat.planeID;
	double threatZEM = greatestThreat.ZEM;
	double timeToGo = greatestThreat.timeToGo;

	/* If there is no plane to avoid, then take Dubin's path to the 
	destination waypoint*/
	if (((threatID < 0) && (threatZEM < 0)) || timeToGo > MINIMUM_TIME_TO_GO) {
		return takeDubinsPath(plane1);
	}

	/* If there is a plane to avoid, then figure out which direction it 
	should turn*/
	bool turnRight = shouldTurnRight(plane1, planes[threatID]);
	
	/* Calculate turning radius to avoid collision*/
	double turningRadius = calculateTurningRadius(threatZEM);

	/* Given turning radius and orientation of the plane, calculate 
	next collision avoidance waypoint*/
	return calculateWaypoint(plane1, turningRadius, turnRight);
}

	
/* Function that returns the ID of the most dangerous neighboring plane and its ZEM. */
real::threatContainer real::findGreatestThreat(PlaneObject &plane1, std::map<int, PlaneObject> &planes) {
    
	/* Set reference for origin (Northwest corner of the course)*/
	real::coordinate origin;
	origin.latitude = 32.606573;
	origin.longitude = -85.490356;
	origin.altitude = 400;
    
	/* Set preliminary plane to avoid as non-existent and most dangerous ZEM as negative */
	int planeToAvoid = -1;
	int iPlaneToAvoid = -1;
	double mostDangerousZEM = -1.0;
	double iMostDangerousZEM = -1.0;
    
	/* Set the preliminary time-to-go high */
	double minimumTimeToGo = MINIMUM_TIME_TO_GO;
	double iMinimumTimeToGo = 3.5;

	/* Declare second plane and ID variable */
	PlaneObject plane2;
	int ID;
    
	/* Make a position vector representation of the current plane */
	double magnitude2, direction2;
	double magnitude = findDistance(origin.latitude, origin.longitude, 
		plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude);
	double direction = findAngle(origin.latitude, origin.longitude, 
		plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude);
	real::mathVector p1(magnitude,direction);

	/* Make a heading vector representation of the current plane */
	real::mathVector d1(1.0,toCartesian(plane1.getCurrentBearing()));
	
	/* Declare variables needed for this loop */
	real::mathVector pDiff, dDiff;
	double timeToGo, zeroEffortMiss, distanceBetween, timeToDest, bearingDiff;
	std::map<int,real::PlaneObject>::iterator it;
    
	for (it=planes.begin() ; it!= planes.end(); it++) {
		/* Unpacking plane to check */		
		ID = (*it).first;
		plane2 = (*it).second;
		
		/* If it's not in the Check Zone, check the other plane */
		distanceBetween = plane1.findDistance(plane2);
		if (distanceBetween > CHECK_ZONE || plane1.getID() == ID) continue;

		/* Making a position vector representation of plane2 */
		magnitude2 = findDistance(origin.latitude, origin.longitude, 
			plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
		direction2 = findAngle(origin.latitude, origin.longitude, 
			plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
		real::mathVector p2(magnitude2,direction2);

		/* Make a heading vector representation of the other plane */
		real::mathVector d2(1.0,toCartesian(plane2.getCurrentBearing()));

		/* Compute time-to-go */
		pDiff = p1-p2;
		dDiff = d1-d2;
		timeToGo = -1.0*pDiff.dotProduct(dDiff)/(MPS_SPEED*dDiff.dotProduct(dDiff));
		

		/* Compute Zero Effort Miss */
		zeroEffortMiss = sqrt(fabs(pDiff.dotProduct(pDiff) + 
			2.0*(MPS_SPEED*timeToGo)*pDiff.dotProduct(dDiff) + 
			pow(MPS_SPEED*timeToGo,2.0)*dDiff.dotProduct(dDiff)));
		
		if( zeroEffortMiss > DANGER_ZEM || (timeToGo > minimumTimeToGo && timeToGo > iMinimumTimeToGo) || timeToGo < 0 ) continue;
        
		//timeToDest = plane1.findDistance(plane1.getDestination().latitude, 
			//plane1.getDestination().longitude) / MPS_SPEED;

		/* If you're close to your destination and the other plane isn't
		much of a threat, then don't avoid it */ 
		//if ( timeToDest < 5.0 && zeroEffortMiss > 3.0*MPS_SPEED ) continue;

		/* If you're likely to zigzag, don't avoid the other plane */
		//bearingDiff = fabs(plane1.getCurrentBearing() - planes[ID].getCurrentBearing());
		//if ( plane1.findDistance(planes[ID]) > 3.5*MPS_SPEED &&  bearingDiff < CHATTERING_ANGLE) continue;

		/* Second Threshold, to prevent planes from flying into others when trying to avoid less imminent collisions */
		/*if ( zeroEffortMiss <= SECOND_THRESHOLD && timeToGo <= iMinimumTimeToGo ) {
			iPlaneToAvoid = ID;
			iMostDangerousZEM = zeroEffortMiss;
			iMinimumTimeToGo = timeToGo;
			continue;
		}*/

		planeToAvoid = ID;
		mostDangerousZEM = zeroEffortMiss;
		minimumTimeToGo = timeToGo;
	}

	real::threatContainer greatestThreat;
	if (iPlaneToAvoid > -1) {
		greatestThreat.planeID = iPlaneToAvoid;
		greatestThreat.ZEM = iMostDangerousZEM;
		greatestThreat.timeToGo = iMinimumTimeToGo;		
	}
	else {
		greatestThreat.planeID = planeToAvoid;
		greatestThreat.ZEM = mostDangerousZEM;
		greatestThreat.timeToGo = minimumTimeToGo;
	}

	return greatestThreat;
}


/* Returns true if the original plane (plane1) should turn right to avoid plane2,
false if otherwise. Takes original plane and its greatest threat as parameters. */
bool real::shouldTurnRight(PlaneObject &plane1, PlaneObject &plane2) {

	/* For checking whether the plane should turn right or left */
	double theta, theta_dot, R;
	double cartBearing1 = toCartesian(plane1.getCurrentBearing());
	double cartBearing2 = toCartesian(plane2.getCurrentBearing());
	double V = MPS_SPEED;
	
	/* Calculate theta, theta1, and theta2. Theta is the cartesian angle
	from 0 degrees (due East) to plane2 (using plane1 as the origin). This 
	may be referred to as the LOS angle. */
	theta = findAngle(plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude, 
		plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
	R = findDistance(plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude, 
		plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
	theta_dot = (V*sin((cartBearing2 - theta)*PI/180) - V*sin((cartBearing1 - theta)*PI/180)) / R;

	return (theta_dot >= 0);
}


/* Calculate the turning radius based on the zero effort miss. */
double real::calculateTurningRadius(double ZEM) {
    
	double l = LAMBDA;
	double ds = DESIRED_SEPARATION;
	return MINIMUM_TURNING_RADIUS*exp(l*ZEM/ds);
}


/* Find the new collision avoidance waypoint for the plane to go to */
real::waypoint real::calculateWaypoint(PlaneObject &plane1, double turningRadius, bool turnRight){

	real::waypoint wp;	
	double V = MPS_SPEED;
	double delta_T = TIME_STEP;	
	double cartBearing = toCartesian(plane1.getCurrentBearing())* PI/180;
	double delta_psi = V / turningRadius * delta_T;
	if (turnRight) delta_psi *= -1.0;
	double psi = (cartBearing + delta_psi);
	wp.longitude = plane1.getCurrentLoc().longitude + V*cos(psi)/DELTA_LON_TO_METERS;
	wp.latitude = plane1.getCurrentLoc().latitude + V*sin(psi)/DELTA_LAT_TO_METERS;
	wp.altitude = plane1.getCurrentLoc().altitude;
    
	return wp;
}


/* This function calculates any maneuvers that are necessary for the 
current plane to avoid looping. Returns a waypoint based on calculations. 
If no maneuvers are necessary, then the function returns the current 
destination. */
real::waypoint real::takeDubinsPath(PlaneObject &plane1) {
    
	/* Initialize variables */
	real::coordinate circleCenter;
	real::waypoint wp = plane1.getDestination();
	double minTurningRadius = 0.75*MINIMUM_TURNING_RADIUS;
	bool destOnRight;
    
	/* Calculate cartesian angle from plane to waypoint */
	double wpBearing = findAngle(plane1.getCurrentLoc().latitude, 
		plane1.getCurrentLoc().longitude, wp.latitude, wp.longitude);
    
	/* Calculate cartesian current bearing of plane (currentBearing is stored as Cardinal) */
	double currentBearingCardinal = plane1.getCurrentBearing();	
	double currentBearingCartesian = toCartesian(currentBearingCardinal);
	
	if (fabs(currentBearingCardinal) < 90.0)
	/* Figure out which side of the plane the waypoint is on */		
		destOnRight = ((wpBearing < currentBearingCartesian) && 
                       (wpBearing > manipulateAngle(currentBearingCartesian - 180.0)));
	else
		destOnRight = !((wpBearing > currentBearingCartesian) && 
                        (wpBearing < manipulateAngle(currentBearingCartesian - 180.0)));
        
	/* Calculate the center of the circle of minimum turning radius on the side that the waypoint is on */
	circleCenter = calculateLoopingCircleCenter(plane1, minTurningRadius, destOnRight);

	/* If destination is inside circle, must fly opposite direction before we can reach destination */
	if (findDistance(circleCenter.latitude, circleCenter.longitude, wp.latitude, wp.longitude) < 
			minTurningRadius)
		return calculateWaypoint(plane1, minTurningRadius, !destOnRight);
	else
		return wp;
}

/* Returns a coordinate of the center of a circle used to prevent looping. */
real::coordinate real::calculateLoopingCircleCenter(PlaneObject &plane, double turnRadius, bool turnRight) {
    
	real::coordinate circleCenter;
	circleCenter.altitude = plane.getCurrentLoc().altitude;
	double angle;
    
	if (turnRight)
		angle = (toCartesian(plane.getCurrentBearing()) - 90 - 22.5) * PI/180.0; 
	else
		angle = (toCartesian(plane.getCurrentBearing()) + 90 + 22.5) * PI/180.0;
    
	double xdiff = turnRadius*cos(angle);
	double ydiff = turnRadius*sin(angle);
	circleCenter.longitude = plane.getCurrentLoc().longitude + xdiff/DELTA_LON_TO_METERS;
	circleCenter.latitude = plane.getCurrentLoc().latitude + ydiff/DELTA_LAT_TO_METERS; 

	return circleCenter;
}
