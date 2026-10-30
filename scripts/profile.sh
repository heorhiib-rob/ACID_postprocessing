cd "$(dirname "$0")"
cd ..
if [ $# -eq 0 ]
then
valgrind --tool=callgrind --dump-instr=yes --collect-jumps=yes ./build/simple_MAPF_solver -m ../../MAPF-benchmark/mapf-map/den520d.map -a ../../MAPF-benchmark/mapf-scen-random/scen-random/den520d-random-0.scen -k 500 -G 0 -i 100 --destroy_operator ADAPTIVE -n 20
else
valgrind --tool=callgrind --dump-instr=yes --collect-jumps=yes ./build/simple_MAPF_solver @
fi
