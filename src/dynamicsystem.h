// Lightweight linear algebra tools
#include "linearalgebra.h"

// Writing to csv file.
#include <fstream>

// Math operations
#include <cmath>

// Used for passing controller to robot instances
#include <functional>

// Base class for all dynamic systems
class dynamicsystem
{
public:
	dynamicsystem(
		std::vector<double>& initialState_IN,
	 	std::vector<double>& initialInput_IN,
	 	std::vector<double>& time_IN, 
		int sLen_IN, 
		int iLen_IN)
	{
		try{
			if(initialState_IN.size() != sLen_IN)
			{
				throw "Dynamic system cannot be created: state dimensions do not match.\n";
			}
			if(initialInput_IN.size() != iLen_IN)
			{
				throw "Dynamic system cannot be created: input dimensions do not match.\n";
			}		

			_sLen = sLen_IN;
			_iLen = iLen_IN;

			_state = std::vector<double>(_sLen);	
			_input = std::vector<double>(_iLen);	

			for (size_t s = 0; s < _sLen; s++) _state[s] = initialState_IN[s];
			for (size_t i = 0; i < _iLen; i++) _input[i] = initialInput_IN[i];				

			_dt = time_IN[0];
			_ft = time_IN[1];

		}
		catch(char const* s)
		{
			std::cout<< s;
		}
	}

	// Change the dynamic system controller
	void SetController(const std::function<std::vector<double>(std::vector<double>&,std::vector<double>&,double)>& newController_IN)
	{
		_controller = newController_IN;
	}

	virtual void SetParameters()
	{
		std::cout << "SetParameters() needs to be defined at derived class.\n"; 
	}

	// Simulation
	void Simulate()
	{
		try{
			if(_controller != NULL)
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
			else
			{
				throw "Simulation can't start: No controller found.\n";
			}			
		}
		catch(char const* s)
		{
			std::cout << s;
		}
	}

	// Reset the system. Clears output vectors.
	void Reset(std::vector<double>& resetState_IN, 
		   std::vector<double>& resetInput_IN)
	{
		_stateVector.clear();
		_inputVector.clear();
		_timeVector.clear();

		for (size_t s = 0; s < _sLen; s++) _state[s] = resetState_IN[s];
		for (size_t i = 0; i < _iLen; i++) _input[i] = resetInput_IN[i];	
	}

	// Export simulation csv file
	void ExportCSV(char const* s)
	{
		try{
			if(_timeVector.size() > 0)
			{
				std::ofstream output_file (s);
				for(size_t r = 0; r<_timeVector.size(); r++)
				{
					output_file << _timeVector[r] << ", ";
					for(double& s: _stateVector[r]) output_file << s << ", ";
					for(auto it = _inputVector[r].begin(); it != _inputVector[r].end(); ++it)
					{
						if(it+1 != _inputVector[r].end())
						{
							output_file << *it << ", ";
						}
						else
						{
							output_file << *it;
						}
					}

					output_file << "\n";
				}

				output_file.close();
			}
			else
			{
				throw "Can't export csv: Run the simulation first.\n";
			}
		}
		catch(char const* s)
		{
			std::cout << s;
		}		
	}	

protected:
	// System Equations: Only diff is protected
	virtual std::vector<double> diff(				// Difference Equation. Defined in derived class.
		std::vector<double>& state_IN, 
		std::vector<double>& input_IN) = 0;		

private:
	// System Equations: Private
	std::vector<double> integrator(
		std::vector<double>& state_IN,
		std::vector<double>& input_IN)				// Simple integrator
	{
		std::vector<double> s_next(_sLen);
		std::vector<double> s_dot = diff(state_IN, input_IN);
		for (size_t s = 0; s < _sLen; s++) s_next[s] = _state[s] + s_dot[s] * _dt;
		return s_next;
	}

	// Controller: Defined in derived class or outside in the main program.
	std::function<std::vector<double>(std::vector<double>&, std::vector<double>&, double)> _controller = NULL;				

	// States & Inputs: current values
	std::vector<double> _state;
	std::vector<double> _input;

	size_t _sLen = 0;		// State vector dimension
	size_t _iLen = 0;		// Input vector dimension

	// Integration
	double _dt = 0.0;		// Integrator delta [s]
	double _ft = 0.0;		// Final time [s]

	// Output 
	std::vector<std::vector<double>> _stateVector;
	std::vector<std::vector<double>> _inputVector;
	std::vector<double> _timeVector;
};
