#define _USE_MATH_DEFINES

// Wheel 
#include "wheel.h"

std::vector<double> SpeedController(std::vector<double>& state_IN, std::vector<double>& input_IN, double time_IN)
{
	if(time_IN > 5.0)
	{
		double target_v = 0.25; // [m/s]
		double target_w = 5.0*(M_PI/180.0); // Set to 5 degree/s, but passed value is in rad/s		

		return {target_v, target_w};
	}
	else
	{
		return input_IN;
	}
}

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

	/* Reset robot at same initial condition */
	myWheel.Reset(state, input);
	myWheel.SetController(&SpeedController); 	// Attach a custom controller
	myWheel.Simulate();
	myWheel.ExportCSV("wheel_output_custom_controller.csv");
}
