// Wheel advanced is derived from dynamic system
#include "dynamicsystem.h"

const size_t NUMBEROFSTATES = 5;
const size_t NUMBEROFINPUTS = 2;

class WheelAdvanced : public dynamicsystem
{
public:
	// Default constructor with custom initial settings
	WheelAdvanced(
		std::vector<double>& initialState_IN, 
		std::vector<double>& initialInput_IN, 
		std::vector<double>& time_IN) : dynamicsystem(
			initialState_IN, 
			initialInput_IN, 
			time_IN, 
			NUMBEROFSTATES, 
			NUMBEROFINPUTS)
	{
		SetController(&ConstantTorqueController);
	}

	// Default controller: same speed
	static std::vector<double> ConstantTorqueController(std::vector<double>& state_IN, std::vector<double>& input_IN, double time_IN)
	{
		return input_IN;
	}

private:
	// Advanced wheel parameters
	double _r = 0.1;			// [m]
	double _m = 0.5;			// [kg]			
	double _I = 0.5*_m*_r*_r;	// [kg*m^2]
	double _b = 0.1;			// [kg*m^2/s]

	std::vector<double> diff(std::vector<double>& state_IN, std::vector<double>& input_IN)
	{
		std::vector<double> s_diff(NUMBEROFSTATES);
		s_diff[0] = state_IN[1]; // alpha_dot
		s_diff[1] = (input_IN[0]-_b*state_IN[1])/_I; // (Torque_IN-b*alpha_dot) / I_disk
		s_diff[2] = cos(state_IN[4])*_r*state_IN[1]; // cos(theta)*r*alpha_dot
		s_diff[3] = sin(state_IN[4])*_r*state_IN[1]; // sin(theta)*r*alpha_dot
		s_diff[4] = state_IN[4]; // w 

		return s_diff;
	}
};
