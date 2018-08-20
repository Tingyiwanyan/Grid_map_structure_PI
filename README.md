# Incremental-MFPT
Reachability and Differential based Heuristics for Solving Markov Decision Processes

i
parameter config file is under ./configs

To install, git Clone external-libs in gitlab, and read INSTALL.txt

In genernal, we need these libs (also see packages.cmake):

1. OpenGL things, such as glut3 etc, main OpenCV modules
2. Matrix libs: armadillo, eigen3
3. Parsing libs: yaml-cpp (tested 0.5.2 and 0.5.3)
4. Netcdf lib: netCDF-4 C++ version 4.2.1 or 4.4.2 (must be c++ version)
5. libgp (modified version in git)

To compile:
  mkdir build
  cd build
  cmake ..
  make

Caveats:
  if roms data is required, download roms from git, put that folder ouside (but side by side with current folder).

