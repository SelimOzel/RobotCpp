// Wheel 
#include "wheel.h"

int main()
{
	std::cout << "Robot Fundamentals: Wheel\n";

	/* Inputs */
	double v = 0.1;		// Wheel velocity [m/s]
	double w = 0.0;		// Wheel angular speed in global frame [rad/s]

	/* State */
	double x = 0.0;		// Wheel x position in global frame [m]
	double y = 0.0;		// Wheel y position in global frame [m]
	double a = 0.0;		// Wheel angle in global frame [rad]

	/* Integrator Values */
	double dt = 0.05;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	/* Wheel construction */
	std::vector<double> state = { x,y,a };
	std::vector<double> input = { v,w };
	std::vector<double> time = { dt,ft };
	
	/* Wheel simulation & data extraction */
	Wheel myWheel(state, input, time);
	myWheel.Simulate();
	myWheel.ExportCSV("wheel_output.csv");
}
