FILE(REMOVE_RECURSE
  "src/real/msg"
  "src/real/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/real/srv/__init__.py"
  "src/real/srv/_SaveFlightData.py"
  "src/real/srv/_RequestWaypointInfo.py"
  "src/real/srv/_CreateSimulatedPlane.py"
  "src/real/srv/_GoToWaypoint.py"
  "src/real/srv/_DeleteSimulatedPlane.py"
  "src/real/srv/_RequestPlaneID.py"
  "src/real/srv/_LoadPath.py"
  "src/real/srv/_LoadCourse.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
