cd "$(dirname "$0")"
cd ..
# disable cpu scaling
sudo cpupower frequency-set --governor performance >/dev/null
# run experiment
./build/sipp_pp_experiment
# add to database
cd ~/Programy/diplomka/LNS4MAPF/experiments
python3 add_all_experiments.py
# run the MAPF LNS 2 experiments
python3 sipp_pp_original_code_test.py
# add to the database
mv results_lns2.csv-PP.csv results_lns2.csv
python3 add_experiment_csv.py
# enable cpu scaling
sudo cpupower frequency-set --governor powersave >/dev/null
# shut down
shutdown now
