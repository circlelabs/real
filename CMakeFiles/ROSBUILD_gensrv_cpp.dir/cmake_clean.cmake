FILE(REMOVE_RECURSE
  "src/real/msg"
  "src/real/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/real/SaveFlightData.h"
  "srv_gen/cpp/include/real/RequestWaypointInfo.h"
  "srv_gen/cpp/include/real/CreateSimulatedPlane.h"
  "srv_gen/cpp/include/real/GoToWaypoint.h"
  "srv_gen/cpp/include/real/DeleteSimulatedPlane.h"
  "srv_gen/cpp/include/real/RequestPlaneID.h"
  "srv_gen/cpp/include/real/LoadPath.h"
  "srv_gen/cpp/include/real/LoadCourse.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
