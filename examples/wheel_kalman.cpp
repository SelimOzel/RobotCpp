#define _USE_MATH_DEFINES

// Wheel 
#include "wheeladvanced.h"

int main()
{
	/* Inputs */
	double torque = 2.0; // Wheel torque [Nm]
	double w = 5.0*(M_PI/180.0); // Wheel angular speed in global frame [rad/s]

	/* State */
	double alpha = 0.0;	// Wheel angle [rad]
	double alpha_dot = 0.0;	// Wheel angular speed [rad/s]	
	double x = 0.0;	// Wheel x position in global frame [m]
	double y = 0.0;	// Wheel y position in global frame [m]
	double a = 0.0;	// Wheel angle in global frame [rad]

	/* Integrator Values */
	double dt = 0.05;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	/* Wheel construction */
	std::vector<double> state = { alpha, alpha_dot,x,y,a };
	std::vector<double> input = { torque,w };
	std::vector<double> time = { dt,ft };
	
	/* Wheel simulation & data extraction */
	WheelAdvanced myWheel(state, input, time);
	myWheel.Simulate();
	myWheel.ExportCSV("wheel_kalman_constant_torque.csv");

	std::cout << "wheel_kalman: Completed\n";
	return 1;
}
