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
		Matrix& initialState_IN,
	 	Matrix& initialInput_IN,
	 	std::vector<double>& time_IN, 
		int sLen_IN, 
		int iLen_IN)
	{
		try{
			if(initialState_IN.Size()[0] != sLen_IN)
			{
				throw "Dynamic system cannot be created: state dimensions do not match.\n";
			}
			if(initialInput_IN.Size()[0] != iLen_IN)
			{
				throw "Dynamic system cannot be created: input dimensions do not match.\n";
			}		

			_sLen = sLen_IN;
			_iLen = iLen_IN;

			_state = initialState_IN;	
			_input = initialInput_IN;				

			_dt = time_IN[0];
			_ft = time_IN[1];

		}
		catch(char const* s)
		{
			std::cout<< s;
		}
	}

	// Change the dynamic system controller
	//void SetController(const std::function<std::vector<double>(std::vector<double>&,std::vector<double>&,double)>& newController_IN)
	void SetController(const std::function<Matrix(Matrix&,Matrix&,double)>& newController_IN)
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
	void Reset(
		Matrix& resetState_IN, 
		Matrix& resetInput_IN)
	{
		_stateVector.clear();
		_inputVector.clear();
		_timeVector.clear();

		_state = resetState_IN;
		_input = resetInput_IN;	
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
					//for(double& s: _stateVector[r]) output_file << s << ", ";
					for(unsigned i = 0; i<_sLen; i++)
					{
						output_file << _stateVector[r](i,0) << ", ";
					}
					//for(auto it = _inputVector[r].begin(); it != _inputVector[r].end(); ++it)
					for(unsigned i = 0; i<_iLen; i++)
					{
						if(i+1 != _iLen)
						{
							output_file << _inputVector[r](i,0) << ", ";
						}
						else
						{
							output_file << _inputVector[r](i,0);
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
	virtual Matrix diff(	// Difference Equation. Defined in derived class.
		Matrix& state_IN, 
		Matrix& input_IN) = 0;		

private:
	// System Equations: Private
	Matrix integrator(		// Simple integrator
		Matrix state_IN,
		Matrix input_IN)	
	{
		Matrix s_dot = diff(state_IN, input_IN);
		return _state + s_dot * _dt;
	}

	// Controller: Defined in derived class or outside in the main program.
	//std::function<std::vector<double>(std::vector<double>&, std::vector<double>&, double)> _controller = NULL;
	std::function<Matrix(Matrix&, Matrix&, double)> _controller = NULL;				

	// States & Inputs: current values
	Matrix _state;
	Matrix _input;

	size_t _sLen = 0;		// State vector dimension
	size_t _iLen = 0;		// Input vector dimension

	// Integration
	double _dt = 0.0;		// Integrator delta [s]
	double _ft = 0.0;		// Final time [s]

	// Output 
	std::vector<Matrix> _stateVector;
	std::vector<Matrix> _inputVector;
	std::vector<double> _timeVector;
};
