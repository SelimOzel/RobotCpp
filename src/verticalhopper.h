/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

// Vertical hopper
#include "dynamicsystem.h"

const size_t NUMBEROFSTATES = 2;
const size_t NUMBEROFINPUTS = 1;

template <class Controller>
class VerticalHopper : public dynamicsystem<Controller>
{
public:
	// Default constructor with custom initial settings
	VerticalHopper(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN) : 
	dynamicsystem<Controller>(initialState_IN, initialInput_IN, time_IN, NUMBEROFSTATES, NUMBEROFINPUTS)
	{
		this->SetEstimator(&ConstantEstimator);
		this->SetController(&ConstantTorqueController);
	}

	// Default estimator: return state as is
	static Matrix ConstantEstimator(Matrix state_IN, Matrix input_IN, double time_IN){return state_IN;}

	// Default controller: constant torque
	static Matrix ConstantTorqueController(Matrix state_IN, Matrix input_IN, double time_IN, Controller& C){return input_IN;}

	// Set Parameters
	void SetParameters(double k_IN, double m_IN, double eq_IN)
	{
		_k = k_IN;
		_m = m_IN;
		_y_eq = eq_IN;
	}

private:
	// Vertical hopper parameters	
	double _g = 9.8; // [m/s^2] gravity
	double _k = 1; // [kg/s^2] linear spring coefficient
	double _m = 1; // [kg] mass	
	double _y_eq = 1; // [m] netural spring length			

	Matrix diff(Matrix state_IN, Matrix input_IN)
	{
		// Hybrid dynamic system: swith based on ground contact
		Matrix s_diff;

		if(state_IN(0,0) > _y_eq)
		{
			s_diff = std::vector<double>
			{
				state_IN(1,0), // y_dot
				-_g // y_dot_dot				
			};
		}
		else
		{
			s_diff = std::vector<double>
			{
				state_IN(1,0), // y_dot
				_k*(_y_eq-state_IN(0,0))/_m // y_dot_dot				
			};			
		}
		return s_diff;
	}
};
