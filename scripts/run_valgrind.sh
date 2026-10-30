#!/bin/bash
cd "$(dirname "$0")"
cd ..

if [ $# -eq 0 ] || [ $1 == "dummy" ]
then
  valgrind --tool=memcheck --leak-check=yes --show-leak-kinds=all --track-origins=yes ./build/simple_MAPF_solver -m ../../my_map/dummy.map -a ../../my_map/dummy.scen -k 1 -G false
elif [ $1 == "den520d" ]
then
valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all --track-origins=yes --gen-suppressions=all ./build/simple_MAPF_solver -m ../../MAPF-benchmark/mapf-map/den520d.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/den520d-random-0.scen -k 500 -G false -i 1 --destroy_operator ADAPTIVE
else
  echo "Invalid argument"
fi
