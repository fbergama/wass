rm -Rf build
mkdir build
cd build
cmake ../src -DCMAKE_INSTALL_PREFIX=$PREFIX
make
make install
cd ..
rm -Rf build
rm -Rf ext
