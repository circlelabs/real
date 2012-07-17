/* Collision Avoidance Node
This node controls the collision avoidance algorithm - an implementation of reactive inverse PN. */

//standard C++ headers
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <queue>
#include <map>
#include <cmath>
#include <stdio.h>
#include <time.h>
#include <boost/lexical_cast.hpp>

//ROS headers
#include "ros/ros.h"
#include "real/TelemetryUpdate.h"
#include "real/GoToWaypoint.h"
#include "real/RequestWaypointInfo.h"
#include "real/standardDefs.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

//our headers
#include "real/planeObject.h"
#include "real/standardFuncs.h"
#include "real/ripna.h"


#define WEST_MOST_LONGITUDE -85.490356
#define NORTH_MOST_LATITUDE 32.606573

#define METERS_TO_LATITUDE (1.0/111200.0)

#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)

//publisher is global so callbacks can access it
ros::Publisher marker_pub;

/* ROS service clients for calling services from the coordinator */
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestWaypointInfoClient;

/* Variables for the number of goToWaypoint services requested, 
the number of planes in the airspace, and the current planes ID */
int count;
std::map<int, real::PlaneObject> planes; /* map of planes in the airspace.  The key is the plane id of the aircraft */

/* This function is run every time new telemetry information from any plane is recieved. With the new telemetry update, 
information about the plane is updated, including bearing, speed, current location, etc. Additionally, we check to see
if the UAV has reached its current destination, and, if so, update the destination of the UAV. After updating, the
calculateForces function is called to find a the new force acting on the UAV; from this new force, a next waypoint is
generated and forwarded to the coordinator. */
void telemetryCallback(const real::TelemetryUpdate::ConstPtr &msg);

int main(int argc, char **argv) {	
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	/* Subscribe to telemetry outputs and create clients for the goToWaypoint and requestWaypointInfo services. */
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	goToWaypointClient = n.serviceClient<real::GoToWaypoint>("go_to_waypoint");
	requestWaypointInfoClient = n.serviceClient<real::RequestWaypointInfo>("request_waypoint_info");

	//initialize counting
	count = 0;	
    
	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}

void telemetryCallback(const real::TelemetryUpdate::ConstPtr &msg) {	
	int planeID = msg->planeID;

	/* Instantiate services and get planeID. */
	real::GoToWaypoint goToWaypointSrv;
	real::GoToWaypoint goToWaypointSrv2;
	real::RequestWaypointInfo requestWaypointInfoSrv;
	

	/* Request this plane's current normal destination. */
	requestWaypointInfoSrv.request.planeID = planeID;
	requestWaypointInfoSrv.request.isAvoidanceWaypoint = false;
	requestWaypointInfoSrv.request.positionInQueue = 0;

	if (!requestWaypointInfoClient.call(requestWaypointInfoSrv)){
		ROS_ERROR("Did not receive a response from the coordinator");
		return;
	}

	/* If the plane has reached its current destination, move on to the next destination waypoint. This does not set
    the destination	of the plane object in the map "planes". */
	if (findDistance(msg->currentLatitude, msg->currentLongitude, 
					requestWaypointInfoSrv.response.latitude, 
					requestWaypointInfoSrv.response.longitude) < COLLISION_THRESHOLD){

		/* Request next normal destination */
		requestWaypointInfoSrv.request.positionInQueue = 1;

		if (!requestWaypointInfoClient.call(requestWaypointInfoSrv)) {
			ROS_ERROR("Did not recieve a response from the coordinator");
			return;
		}
	}


	/* If the plane is not in our map of planes and has destination waypoints, then add it as a new plane to our map of planes. */
	if (planes.find(planeID) == planes.end() && msg->currentWaypointIndex != -1){ 
		/* This is a new plane, so create a new planeObject and give it the appropriate information. */
		real::PlaneObject newPlane(MPS_SPEED, *msg); 
		planes[planeID] = newPlane; //put the new plane in the map

		/* Update the destination of the PlaneObject with the value found with the requestWaypointInfoSrv call. */
		real::waypoint newDest; 
		newDest.latitude = requestWaypointInfoSrv.response.latitude;
		newDest.longitude = requestWaypointInfoSrv.response.longitude;
		newDest.altitude = requestWaypointInfoSrv.response.altitude;

		planes[planeID].setDestination(newDest);
	}
    /* Else if the plane is not in our map of planes and does not have waypoints, return and do nothing more. */
	else if (planes.find(planeID) == planes.end()) 
		return; 
    

	/* Note: The requestWaypointInfo service returns a waypoint of -1000, -1000 when the UAV cannot retrieve a
    destination from queue. */

	/* If the plane has no waypoint to go to, put it far from all others. */
	if (requestWaypointInfoSrv.response.latitude == -1000){ //plane has no waypoints to go to
		/* Remove in real flights*/
		planes[planeID].setCurrentLoc(-1000,-1000,400);
		/* update the time of last update for this plane to acknowledge 
		it is still in the air */
		planes[planeID].updateTime(); 
		return; 
	}
    
	/* Else (the plane does have a waypoint to go to) update the plane with new position and destination received from
    requestWaypointInfoSrv response. */
	else {
		planes[planeID].update(*msg); //update plane with new position
		real::waypoint newDest;

		newDest.latitude = requestWaypointInfoSrv.response.latitude;
		newDest.longitude = requestWaypointInfoSrv.response.longitude;
		newDest.altitude = requestWaypointInfoSrv.response.altitude;

		planes[planeID].setDestination(newDest); //update plane destination
	}

    /* This code calls the collision avoidance algorithm and determines if collision avoidance maneuvers should be
    taken. Returns a waypoint for the plane to go to. */	
    real::waypoint newWaypoint = findNewWaypoint(planes[planeID], planes);

    if (planes[planeID].getDestination().latitude == newWaypont.latitude &&
        planes[planeID].getDestination().longitude == newWaypont.longitude) continue;

    /* Fill in goToWaypointSrv request with new waypoint information. */
    goToWaypointSrv.request.planeID = planeID;
    goToWaypointSrv.request.latitude = newWaypoint.latitude;
    goToWaypointSrv.request.longitude = newWaypoint.longitude;
    goToWaypointSrv.request.altitude = newWaypoint.altitude;
    goToWaypointSrv.request.isAvoidanceManeuver = true; 
    goToWaypointSrv.request.isNewQueue = true;

	}

}