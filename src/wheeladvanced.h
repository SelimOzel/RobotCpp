/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

// Wheel advanced is derived from dynamic system
#include "dynamicsystem.h"

const size_t NUMBEROFSTATES = 5;
const size_t NUMBEROFINPUTS = 2;

template <class Controller, class Estimator>
class WheelAdvanced : public dynamicsystem<Controller, Estimator>
{
public:
	// Default constructor with custom initial settings
	WheelAdvanced(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN) : 
	dynamicsystem<Controller, Estimator>(initialState_IN, initialInput_IN, time_IN, NUMBEROFSTATES, NUMBEROFINPUTS)
	{
		this->SetEstimator(&ConstantEstimator);
		this->SetController(&ConstantTorqueController);
	}

	// Default estimator: return state as is
	static Matrix ConstantEstimator(Matrix state_IN, Matrix input_IN, double time_IN){return state_IN;}

	// Default controller: constant torque
	static Matrix ConstantTorqueController(Matrix state_IN, Matrix input_IN, double time_IN, Controller& C){return input_IN;}

	// Set Parameters
	void SetParameters(double r_IN, double m_IN, double b_IN)
	{
		_r = r_IN;
		_m = m_IN;
		_I = 0.5*_m*_r*_r;
		_b = b_IN;
	}

private:
	// Advanced wheel parameters
	double _r = 1.0;			// [m]
	double _m = 1.0;			// [kg]			
	double _I = 0.5*_m*_r*_r;	// [kg*m^2]
	double _b = 0.5;			// [kg*m^2/s]

	Matrix diff(Matrix state_IN, Matrix input_IN)
	{
		// Advanced wheel equations of motion
		Matrix s_diff;
		s_diff = std::vector<double>
		{
			state_IN(1,0), // alpha_dot
			(input_IN(0,0)-_b*state_IN(1,0))/_I, // (Torque_IN-b*alpha_dot) / I_disk
			cos(state_IN(3,0))*_r*state_IN(1,0), // cos(theta)*r*alpha_dot
			sin(state_IN(3,0))*_r*state_IN(1,0), // sin(theta)*r*alpha_dot
			input_IN(1,0) // w 
		};
		return s_diff;
	}
};
