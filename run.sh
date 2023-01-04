mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j6
./raytracer 