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
#include "wheeladvanced.h"
#include "controllers.h"

KalmanFilter AdvancedWheelKalman(Matrix& initialState_IN)
{
	// Initialize Kalman Filter ...
	// 4x4 correlation matrix. Initialized to 10.0
	Matrix P(4,10); 

	// 4x4 Prediction matrix.
	Matrix F(4,4,0.0);
	double dt = 0.05;	// Integrator delta [s].. SAME AS INTEGRATOR
	F = {
		{1,0,dt,0},
		{0,1,0,dt},
		{0,0,1,0},
		{0,0,0,1}
	};

	// 4x2 input matrix not used.
	Matrix B(4,2,0.0);

	// 2x4 sensor matrix
	Matrix H(2,4,0.0);
	H = {
		{1,0,0,0},
		{0,1,0,0}
	};

	Matrix kalmanStateInit(4,0,0.0);
	kalmanStateInit.Set(0,0, initialState_IN(0,0)); // Wheel angle, alpha
	kalmanStateInit.Set(0,0, initialState_IN(4,0)); // Wheel global angle, theta

	KalmanFilter Kalman;
	Kalman.Initialize(F, H, B, kalmanStateInit, P);

	return Kalman;
}

Matrix GlobalPositionEstimator(Matrix& state_IN, Matrix& input_IN, double time_IN)
{
	// 2x1 measurement vector
	Matrix z(2,1,0);
	z.Set(0,0, state_IN(0,0)); // Save wheel angle, alpha
	z.Set(1,0, state_IN(4,0)); // Save wheel global angle, theta

	// 2x1 input vector. set to zero.
	Matrix u(2,1,0);

	//Kalman.Filter(z,u);
	//KalmanFilter wheelKalman(F, H, B);

	return state_IN;
}

int main()
{
	// Inputs 
	double torque = 5.0; // Wheel torque [Nm]
	double w = 5.0*(M_PI/180.0); // Wheel angular speed in global frame [rad/s]

	// State 
	double alpha = 0.0;	// Wheel angle [rad]
	double alpha_dot = 0.0;	// Wheel angular speed [rad/s]	
	double x = 0.0;	// Wheel x position in global frame [m]
	double y = 0.0;	// Wheel y position in global frame [m]
	double a = 0.0;	// Wheel angle in global frame [rad]

	// Integrator Values 
	double dt = 0.05;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	// Wheel construction 
	Matrix state(std::vector<double> { alpha,alpha_dot,x,y,a });
	Matrix input(std::vector<double> { torque,w });
	std::vector<double> time = { dt,ft };
	
	// Wheel simulation & data extraction 
	WheelAdvanced<NOCONTROLLER> myWheel(state, input, time);
	myWheel.SetEstimator(&GlobalPositionEstimator); // Set the kalman filter for global position estimation
	myWheel.Simulate();
	myWheel.ExportCSV("wheel_1kg_kalman_constant_torque.csv");

	// Reset and start with a different mass
	myWheel.Reset(state, input);
	myWheel.SetParameters(1.0, 2.0, 0.5);
	myWheel.Simulate();
	myWheel.ExportCSV("wheel_2kg_kalman_constant_torque.csv");

	std::cout << "wheel_kalman: Completed\n";
	return 1;
}
