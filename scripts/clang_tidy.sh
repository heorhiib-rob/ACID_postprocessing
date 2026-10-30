cd "$(dirname "$0")"
cd ..
clang-tidy src/*.cpp -p build --fix
