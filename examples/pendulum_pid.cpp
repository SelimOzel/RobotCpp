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
#include "pendulum.h"

int main()
{
	// Inputs 
	double a = 0.0; // angular acceleration [rad/s^2]

	// State 
	double theta = 90.0*(M_PI/180.0); // pendulum angle [rad]
	double theta_dot = 0.0;	// pendulum angular speed [rad/s]	

	// Integrator Values 
	double dt = 0.001;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	// Pendulum construction 
	Matrix state(std::vector<double> { theta,theta_dot});
	Matrix input(std::vector<double> { a });
	std::vector<double> time = { dt,ft };

	// Peundulum simulation & data extraction 
	Pendulum myPendulum(state, input, time);
	myPendulum.SetParameters(0.1, 0.1); // length 1m, damping 0.0
	myPendulum.Simulate();
	myPendulum.ExportCSV("pendulum_pid.csv");	

	return 1;
}