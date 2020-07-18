/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

// Double pendulum
#include "dynamicsystem.h"

const size_t NUMBEROFSTATES = 4;
const size_t NUMBEROFINPUTS = 2;

template <class Controller, class Estimator>
class DoublePendulum : public dynamicsystem<Controller, Estimator>
{
public:
	// Default constructor with custom initial settings
	DoublePendulum(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN) : 
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
	void SetParameters(double l1_IN, double l2_IN, double m1_IN, double m2_IN, double b1_IN, double b2_IN)
	{
		_l1 = l1_IN;
		_l2 = l2_IN;
		_m1 = m1_IN;
		_m2 = m2_IN;
		_b1 = b1_IN;
		_b2 = b2_IN;
	}

private:
	// Double pendulum parameters
	double _l1 = 1.0;			// [m]		
	double _l2 = 1.0;
	double _m1 = 1.0; 			// [kg]
	double _m2 = 1.0; 			
	double _g = 9.8;			// [m/s^2]
	double _b1 = 0.5;			// [1/s]
	double _b2 = 0.5;			

	Matrix diff(Matrix state_IN, Matrix input_IN)
	{
		// Double pendulum equations of motion
		Matrix s_diff;
		s_diff = std::vector<double>
		{
			state_IN(1,0), // theta1_dot
			( -_g*(2*_m1+_m2)*sin(state_IN(0,0)) - _m2*_g*sin(state_IN(0,0)-2*state_IN(2,0)) - 2*sin(state_IN(0,0)-state_IN(2,0))*_m2*(state_IN(3,0)*state_IN(3,0)*_l2+state_IN(1,0)*state_IN(1,0)*_l1*cos(state_IN(0,0)-state_IN(2,0))) )/_l1*(2*_m1+_m2-_m2*cos(2*state_IN(0,0)-2*state_IN(2,0))), // theta1_dot_dot
			state_IN(3,0), // theta2_dot
			( 2*sin(state_IN(0,0)-state_IN(2,0))*(state_IN(1,0)*state_IN(1,0)*_l1*(_m1+_m2)+_g*(_m1+_m2)*cos(state_IN(0,0))+state_IN(2,0)*state_IN(2,0)*_l2*_m2*cos(state_IN(0,0)-state_IN(2,0))) )/_l2*(2*_m1+_m2-_m2*cos(2*state_IN(0,0)-2*state_IN(2,0)))// theta2_dot_dot
		};
		return s_diff;
	}
};
