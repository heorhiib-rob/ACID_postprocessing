cd "$(dirname "$0")"
cd ..
# disable cpu scaling
sudo cpupower frequency-set --governor performance >/dev/null
./build/tests/$1
# enable cpu scaling
sudo cpupower frequency-set --governor powersave >/dev/null
