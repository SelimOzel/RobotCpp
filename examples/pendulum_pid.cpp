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

Matrix PIDControllerCallBack(Matrix state_IN, Matrix input_IN, double time_IN, PID& controller_IN)
{
	double referenceAngle = 45.0*(M_PI/180.0);
	double error =  referenceAngle - state_IN(0,0);
	double error_dot = -state_IN(1,0);
	Matrix u(std::vector<double> { controller_IN.compute(error, error_dot) });
	return u;
}

int main()
{
	// Inputs 
	double a = 0.0; // angular acceleration [rad/s^2]

	// State 
	double theta = 25.0*(M_PI/180.0); // pendulum angle [rad]
	double theta_dot = 0.0;	// pendulum angular speed [rad/s]	

	// Integrator Values 
	double dt = 0.001;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	// PID Controller
	double kp = 50.0;
	double kd = 45;
	double ki = 4.0;
	int integrator_length = 100;
	PID pendulumPID(kp, kd, ki, integrator_length);

	// Pendulum initial values
	Matrix state(std::vector<double> { theta,theta_dot});
	Matrix input(std::vector<double> { a });
	std::vector<double> time = { dt,ft };

	// Peundulum creation
	Pendulum<PID, NOESTIMATOR> myPendulum(state, input, time, pendulumPID);
	myPendulum.SetParameters(1.0, 1.0); // length 1m, damping 0.0
	myPendulum.SetController(&PIDControllerCallBack); // attach pid controller
	
	// Simulation & data extraction 
	myPendulum.Simulate();
	myPendulum.ExportCSV("pendulum_pid.csv");	

	std::cout << "pendulum_pid: Completed\n";
	return 1;
}