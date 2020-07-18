/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

// Pendulum
#include "dynamicsystem.h"

const size_t NUMBEROFSTATES = 2;
const size_t NUMBEROFINPUTS = 1;

template <class Controller, class Estimator>
class Pendulum : public dynamicsystem<Controller, Estimator>
{
public:
	// Default constructor with custom initial settings
	Pendulum(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, Controller& C) : 
	dynamicsystem<Controller, Estimator>(initialState_IN, initialInput_IN, time_IN, NUMBEROFSTATES, NUMBEROFINPUTS, C)
	{
		this->SetEstimator(&ConstantEstimator);
		this->SetController(&ConstantTorqueController);
	}

	// Default estimator: return state as is
	static Matrix ConstantEstimator(Matrix state_IN, Matrix input_IN, double time_IN){return state_IN;}

	// Default controller: constant torque
	static Matrix ConstantTorqueController(Matrix state_IN, Matrix input_IN, double time_IN, Controller& C){return input_IN;}

	// Set Parameters
	void SetParameters(double r_IN, double b_IN)
	{
		_r = r_IN;
		_b = b_IN;
	}

private:
	// Pendulum parameters
	double _r = 1.0;			// [m]		
	double _g = 9.8;			// [m/s^2]
	double _b = 0.5;			// [1/s]

	Matrix diff(Matrix state_IN, Matrix input_IN)
	{
		// Pendulum equations of motion
		Matrix s_diff;
		s_diff = std::vector<double>
		{
			state_IN(1,0), // theta_dot
			-_g/_r*sin(state_IN(0,0))-_b*state_IN(1,0)+input_IN(0,0) // theta_dot_dot
		};
		return s_diff;
	}
};
