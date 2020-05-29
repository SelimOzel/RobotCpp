// Wheel is derived from dynamic system
#include "dynamicsystem.h"

const size_t NUMBEROFSTATES = 3;
const size_t NUMBEROFINPUTS = 2;

class Wheel : public dynamicsystem
{
public:
	// Default constructor with custom initial settings
	Wheel(std::vector<double>& initialState_IN, 
		std::vector<double>& initialInput_IN, 
		std::vector<double>& time_IN) : dynamicsystem(initialState_IN, 
											initialInput_IN, 
											time_IN, 
											NUMBEROFSTATES, 
											NUMBEROFINPUTS)
	{
		SetController(&ConstantSpeedController);
	}

	// Default controller: same speed
	static std::vector<double> ConstantSpeedController(std::vector<double>& state_IN, std::vector<double>& input_IN, double time_IN)
	{
		return input_IN;
	}

private:
	std::vector<double> diff(std::vector<double>& state_IN, std::vector<double>& input_IN)
	{
		std::vector<double> s_diff(NUMBEROFSTATES);
		s_diff[0] = cos(state_IN[2]) * input_IN[0]; // cos(theta) * v
		s_diff[1] = sin(state_IN[2]) * input_IN[0]; // sin(theta) * v
		s_diff[2] = input_IN[1];					// w
		return s_diff;
	}
};