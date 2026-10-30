#!/bin/bash

# Resolve the full path of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR"
cd ..
mkdir build
cd build
echo "Build dir ${PWD}"

# cd "$(dirname "$0")"
# cd ../build
# export CC=/usr/bin/clang
# export CXX=/usr/bin/clang++-18
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

# check whether RELEASE or DEBUG
if [ $# -eq 0 ] || [ $1 == "relwithdebinfo" ]
then
  BUILD_TYPE=RelWithDebInfo
elif [ $1 == "release" ]
then
  BUILD_TYPE=Release
elif [ $1 == "debug" ]
then
  BUILD_TYPE=Debug
fi

# check whether to compile with tests

echo "Build type: ${BUILD_TYPE}"

if [ $# -lt 2 ] || [ $2 == "tests" ]
then
  TESTS=ON
elif [ $2 == "notest" ]
then
  TESTS=OFF
  echo "Disabling tests."
else
  echo "Invalid second argument, should be tests/notests"
fi

cmake .. -DENABLE_TESTS=${TESTS} -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
cmake --build . 

