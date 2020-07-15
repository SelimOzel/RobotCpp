/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

#define _USE_MATH_DEFINES

// Vertical hopper
#include "verticalhopper.h"

int main()
{
	// Inputs 
	double a = 0.0; // vertical acceleration [m/s^2]

	// State 
	double y = 10; // vertical position [m]
	double y_dot = 0.0; // vertical speed [m/s]	

	// Integrator Values 
	double dt = 0.001;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	// Vertical hopper initial values
	Matrix state(std::vector<double> { y,y_dot });
	Matrix input(std::vector<double> { a });
	std::vector<double> time = { dt,ft };

	// Vertical hopper creation
	VerticalHopper<NOCONTROLLER> verticalHopper(state, input, time);
	verticalHopper.SetParameters(1.0, 1.0, 1.0); // k,m,eq

	// Simulation & data extraction 
	verticalHopper.Simulate();
	verticalHopper.ExportCSV("hopper_vertical.csv");	

	std::cout << "hopper_vertical: Completed\n";
	return 1;
}