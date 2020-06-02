// Wheel is derived from dynamic system
#include "dynamicsystem.h"

const size_t NUMBEROFSTATES = 3;
const size_t NUMBEROFINPUTS = 2;

class Wheel : public dynamicsystem
{
public:
	// Default constructor with custom initial settings
	Wheel(
		Matrix& initialState_IN, 
		Matrix& initialInput_IN, 
		std::vector<double>& time_IN) : dynamicsystem(
			initialState_IN, 
			initialInput_IN, 
			time_IN, 
			NUMBEROFSTATES, 
			NUMBEROFINPUTS)
	{
		SetController(&ConstantSpeedController);
	}

	// Default controller: same speed
	static Matrix ConstantSpeedController(Matrix& state_IN, Matrix& input_IN, double time_IN)
	{
		return input_IN;
	}

private:
	Matrix diff(Matrix& state_IN, Matrix& input_IN)
	{
		// Wheel equations of motion
		Matrix s_diff;
		s_diff = std::vector<double>
		{
			cos(state_IN(2,0)) * input_IN(0,0), // cos(theta) * v
			sin(state_IN(2,0)) * input_IN(0,0), // sin(theta) * v
			input_IN(1,0) // w
		}; 
		//std::cout<<input_IN(0,0)<<"\n";
		//std::cout<<input_IN(1,0)<<"\n";
		//Matrix::Print(input_IN);
		return s_diff; // return as column
	}
};
