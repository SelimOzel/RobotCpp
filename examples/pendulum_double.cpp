/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

#define _USE_MATH_DEFINES

// Double pendulum
#include "doublependulum.h"

int main()
{
	// Inputs 
	double a1 = 0.0; // angular acceleration [rad/s^2]
	double a2 = 0.0;

	// State 
	double theta1 = 25.0*(M_PI/180.0); // pendulum angle [rad]
	double theta1_dot = 0.0;	// pendulum angular speed [rad/s]	
	double theta2 = 15.0*(M_PI/180.0); 
	double theta2_dot = 0.0;	

	// Integrator Values 
	double dt = 0.001;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	// Double pendulum initial values
	Matrix state(std::vector<double> { theta1,theta1_dot,theta2,theta2_dot});
	Matrix input(std::vector<double> { a1,a2 });
	std::vector<double> time = { dt,ft };

	// Double pendulum creation
	DoublePendulum<> doublePendulum(state, input, time);
	doublePendulum.SetParameters(1.0, 1.0, 1.0, 1.0 ,0.5, 0.5); // l1,l2,m1,m2,b1,b2

	// Simulation & data extraction 
	doublePendulum.Simulate();
	doublePendulum.ExportCSV("pendulum_double.csv");	

	std::cout << "pendulum_double: Completed\n";
	return 1;
}