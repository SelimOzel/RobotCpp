/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

#define _USE_MATH_DEFINES

// Wheel 
#include "wheel.h"
#include "controllers.h"

Matrix SpeedControllerCallback(Matrix& state_IN, Matrix& input_IN, double time_IN, PID& controller_IN)
{
	if(time_IN > 5.0)
	{
		double target_v = 0.25; // [m/s]
		double target_w = 5.0*(M_PI/180.0); // Set to 5 degree/s, but passed value is in rad/s			
		Matrix Result(std::vector<double> { target_v,target_w });
		return Result;
	}
	else
	{
		return input_IN;
	}
}

int main()
{
	// Inputs 
	double v = 0.1;		// Wheel velocity [m/s]
	double w = 0.0;		// Wheel angular speed in global frame [rad/s]

	// State 
	double x = 0.0;		// Wheel x position in global frame [m]
	double y = 0.0;		// Wheel y position in global frame [m]
	double a = 0.0;		// Wheel angle in global frame [rad]

	// Integrator Values 
	double dt = 0.05;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	// Wheel construction 
	Matrix state(std::vector<double> { x,y,a }); // 3x1 state
	Matrix input(std::vector<double> { v,w }); // 2x1 input
	std::vector<double> time = { dt,ft };
	
	// Wheel simulation & data extraction 
	PID notUsed;
	Wheel<PID> myWheel(state, input, time, notUsed);

	myWheel.Simulate();
	myWheel.ExportCSV("wheel_output.csv");

	// Reset robot at same initial condition 
	myWheel.Reset(state, input);
	myWheel.SetController(&SpeedControllerCallback); 	// Attach a custom controller
	myWheel.Simulate();
	myWheel.ExportCSV("wheel_output_custom_controller.csv");

	std::cout << "wheel_simple: Completed\n";
	
	return 1;
}
