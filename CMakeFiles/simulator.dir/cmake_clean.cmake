FILE(REMOVE_RECURSE
  "src/real/msg"
  "src/real/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/simulator.dir/src/simulator.o"
  "CMakeFiles/simulator.dir/src/real/SimulatedPlane.o"
  "CMakeFiles/simulator.dir/src/real/standardDefs.o"
  "bin/simulator.pdb"
  "bin/simulator"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/simulator.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
