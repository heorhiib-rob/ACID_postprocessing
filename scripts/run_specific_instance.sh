#!/bin/bash
cd "$(dirname "$0")"
cd ..

# check which from the predefined instances to run
if [ $# -eq 0 ] || [ $1 == "den520d" ]
then
./build/simple_MAPF_solver -m ../../MAPF-benchmark/mapf-map/den520d.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/den520d-random-0.scen -k 500 $@
elif [ $1 == "dummy" ]
then 
./build/simple_MAPF_solver -m ../../my_map/dummy.map -a ../../my_map/dummy.scen -k 1 -n 1 -i 0 $@ 
elif [ $1 == "empty" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/empty-16-16.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/empty-16-16-random-1.scen -k 60 $@
elif [ $1 == "empty-32-32" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/empty-32-32.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/empty-32-32-random-1.scen -k 400 $@
elif [ $1 == "random-32-32-10" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/random-32-32-10.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/random-32-32-10-random-1.scen -k 100 $@
elif [ $1 == "empty-8-8" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/empty-8-8.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/empty-8-8-random-1.scen -k 32 -i 500 $@
elif [ $1 == "Paris" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/Paris_1_256.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/Paris_1_256-random-1.scen -k 700 -i 500 $@
elif [ $1 == "ost" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/ost003d.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/ost003d-random-7.scen -k 400 -s 692469116 -i 500 --destroy_operator BLOCKED --sipp_implementation SIPP_suboptimal $@
elif [ $1 == "warehouse" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/warehouse-10-20-10-2-1.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/warehouse-10-20-10-2-1-random-2.scen -k 400 -s 1 -i 500 --destroy_operator BLOCKED --sipp_implementation SIPP_suboptimal $@
elif [ $1 == "room" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/room-64-64-8.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/room-64-64-8-random-2.scen -k 500 -s 1 -i 150 --destroy_operator BLOCKED --sipp_implementation SIPP_suboptimal $@
elif [ $1 == "den312d" ]
then
./scripts/run.sh -m ../../MAPF-benchmark/mapf-map/den312d.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/den312d-random-1.scen -k 250 -s 0 -i 500 --destroy_operator BLOCKED --sipp_implementation SIPP_suboptimal $@
else 
  echo "Unknown instance"
fi
