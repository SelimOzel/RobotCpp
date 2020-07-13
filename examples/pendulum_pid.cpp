/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

#define _USE_MATH_DEFINES

// Pendulum with pid controller 
#include "pendulum.h"
#include "controllers.h"

Matrix PIDControllerCallBack(Matrix& state_IN, Matrix& input_IN, double time_IN, PID& controller_IN)
{
	return input_IN;
}

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

	// PID Controller
	double kp = 0.1;
	double kd = 0.0;
	double ki = 0.0;
	int integrator_length = 100;
	PID pendulumPID(kp, kd, ki, integrator_length);

	// Pendulum initial values
	Matrix state(std::vector<double> { theta,theta_dot});
	Matrix input(std::vector<double> { a });
	std::vector<double> time = { dt,ft };

	// Peundulum creation
	Pendulum<PID> myPendulum(state, input, time, pendulumPID);
	myPendulum.SetParameters(0.1, 0.1); // length 1m, damping 0.0
	myPendulum.SetController(&PIDControllerCallBack); // attach pid controller
	
	// Simulation & data extraction 
	myPendulum.Simulate();
	myPendulum.ExportCSV("pendulum_pid.csv");	

	return 1;
}