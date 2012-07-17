/* SimulatedPlane
This class contains the data structures and functions required to perform plane Simulation.
Each object instantiated will be considered one "plane" in the system. */

#ifndef SIMULATED_PLANE_H
#define SIMULATED_PLANE_H

#define MAXIMUM_TURNING_ANGLE 22.5 //degrees

#include "real/standardDefs.h"
#include "real/Command.h"
#include "real/TelemetryUpdate.h"
#include "real/CreateSimulatedPlane.h"

namespace real {
	class SimulatedPlane {
        private:
            //last received command info
            real::Command lastCommand;
            
            //current information (used mostly in update)
            long long int planeID;
            
            real::waypoint currentLocation;
            real::waypoint currentDest;
            
            //these two values are sent in the telemetry update
            double groundSpeed;
            double bearing;
            
            //this is stored as part of the UAV info
            double actualBearing;
            
            long long int currentWaypointIndex;
            double distanceToDestination;
            
            //index of sent message
            int updateIndex;
            
        public:
            //dummy constructor, shouldn't really be used
            SimulatedPlane();
            
            //primary constructor
            SimulatedPlane(long long int planeID, real::CreateSimulatedPlane::Request &requestFromUser);
        
            //function for handling a command from the coordinator
            bool handleNewCommand(real::Command newCommand);
            
            //periodic function for filling in a new telemetry update for this UAV
            bool fillTelemetryUpdate(real::TelemetryUpdate *tUpdate);
	};
}

#endif