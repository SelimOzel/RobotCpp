# RobotCpp

| Development Environment 	| g++	| Make
| ------------- 			| ------------- | -----
| Windows 10     			| 9.2.0 | GNU Make 3.81

This is a dynamic simulator targeting robot control applications. It is written in C++. The underlying dynamic systems library handles integration. Example classes are provided and they include equations of motions for different types of robots (e.g. simple wheel, pendulum, a vertical hopper ...). New controllers can be attached to example classes. Outputs are saved as csv files. The project contains html+plotly based visualization for some of the examples.

## Development Environment Setup
1. Install GNU Make from [sourceforge](http://gnuwin32.sourceforge.net/packages/make.htm).
2. Install Minimalist GNU for Windows from the organization's [page](http://www.mingw.org/wiki/Install_MinGW).
3. Verify both installations on Windows 10 command line `g++ --version` and `make --version`.

## Building & Visualizing Examples
1. Open Windows 10 command line and change directory to root of the project.
2. Execute `make` to build all examples.
3. Execute `make wheel_simple` to build a specific example. 
4. Change directory to _bin. Execute `wheel_simple.exe`. It will create an output csv.
5. Open `figures\wheel_simple.html` to visualize output data.
6. The binaries can be cleaned with `make clean`.
