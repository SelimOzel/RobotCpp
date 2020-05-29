// I/O Stream
#include <iostream>

// Vector containing all sorts of system variables
#include <vector>

// Math operations
#include <cmath>

// Used for passing controller to robot instances
#include <functional>

class dynamicsystem
{
public:
	// Change the dynamic system controller
	void SetController(const std::function< std::vector<double>(
		std::vector<double>&,
		std::vector<double>&,
		double) >& newController_IN)
	{
		_controller = newController_IN;
	}

	// Simulation
	void Simulate()
	{
		double currentTime = 0.0;

		// Fill initial values
		_stateVector.push_back(_state);
		_inputVector.push_back(_input);
		_timeVector.push_back(currentTime);

		// Run the simulation
		while (currentTime <= _ft)
		{
			// Compute next input and next state. Input must be computed first!
			_input = _controller(_state, _input, currentTime);
			_state = integrator(_state, _input);

			// Update simulation time
			currentTime += _dt;

			// Fill updated values
			_stateVector.push_back(_state);
			_inputVector.push_back(_input);
			_timeVector.push_back(currentTime);
		}
	}

	// Output can be exported after simulation is run.

protected:
	// System Equations
	virtual std::vector<double> diff(				// Difference Equation. Defined in derived class.
		std::vector<double>& state_IN, 
		std::vector<double>& input_IN) = 0;		

	std::vector<double> integrator(std::vector<double>& state_IN,
		std::vector<double>& input_IN)				// Simple integrator
	{
		std::vector<double> s_next(state_IN.size());
		std::vector<double> s_dot = diff(state_IN, input_IN);
		for (size_t s = 0; s < state_IN.size(); s++) s_next[s] = _state[s] + s_dot[s] * _dt;
		return s_next;
	}

	std::function<std::vector<double>(
		std::vector<double>&, 
		std::vector<double>&,
		double)> _controller;						// Controller. Defined in derived class or outside in the main program.

	// States & Inputs: current values
	std::vector<double> _state;
	std::vector<double> _input;

	// Integration
	double _dt;		// Integrator delta [s]
	double _ft;		// Final time [s]

	// Output 
	std::vector<std::vector<double>> _stateVector;
	std::vector<std::vector<double>> _inputVector;
	std::vector<double> _timeVector;
};

class Wheel : public dynamicsystem
{
public:
	// Default constructor
	Wheel(void)
	{
		_state = std::vector<double>(_sLen);	// [ {0}xPos[m], {1}yPos[m], {2}theta[rad] ]
		for (int s = 0; s < _sLen; s++) _state[s] = 0.0;

		_input = std::vector<double>(_iLen);	// [ {0}v[m/s], {1}w[rad/s] ]
		for (int i = 0; i < _iLen; i++) _input[i] = 0.0;

		_dt = 0.1;
		_ft = 5.0;

		SetController(&ConstantSpeedController);
	}

	// Overloaded constructor with custom initial settings
	Wheel(std::vector<double>& initialState_IN, std::vector<double>& initialInput_IN, std::vector<double>& time_IN) : Wheel()
	{
		for (int s = 0; s < _sLen; s++) _state[s] = initialState_IN[s];
		for (int i = 0; i < _iLen; i++) _input[i] = initialInput_IN[i];

		_dt = time_IN[0];
		_ft = time_IN[1];
	}

	// Default controller: same speed
	static std::vector<double> ConstantSpeedController(std::vector<double>& state_IN, std::vector<double>& input_IN, double time_IN)
	{
		return input_IN;
	}

private:
	const int _sLen = 3;	// State vector dimension
	const int _iLen = 2;	// Input vector dimension

	std::vector<double> diff(std::vector<double>& state_IN, std::vector<double>& input_IN)
	{
		std::vector<double> s_diff(state_IN.size());
		s_diff[0] = cos(state_IN[2]) * input_IN[0]; // cos(theta) * v
		s_diff[1] = sin(state_IN[2]) * input_IN[0]; // sin(theta) * v
		s_diff[2] = input_IN[1];					// w
		return s_diff;
	}
};

int main()
{
	std::cout << "Robot Fundamentals: Wheel\n";

	/* Inputs */
	double v = 0.1;		// Wheel velocity [m/s]
	double w = 0.0;		// Wheel angular speed in global frame [rad/s]

	/* State */
	double x = 0.0;		// Wheel x position in global frame [m]
	double y = 0.0;		// Wheel y position in global frame [m]
	double a = 0.0;		// Wheel angle in global frame [rad]

	/* Integrator Values */
	double dt = 0.05;	// Integrator delta [s]
	double ft = 10.0;	// Integrator end time [s]

	/* Wheel construction */
	std::vector<double> state = { x,y,a };
	std::vector<double> input = { v,w };
	std::vector<double> time = { dt,ft };
	
	Wheel myWheel(state, input, time);
	myWheel.Simulate();
}
