/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

#ifndef __DYNAMICSYSTEM_CPP
#define __DYNAMICSYSTEM_CPP

#include "dynamicsystem.h"

template<class Controller>
dynamicsystem<Controller>::dynamicsystem(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN)
{
	initializestate(initialState_IN, initialInput_IN, time_IN, sLen_IN, iLen_IN);
}

template<class Controller>
dynamicsystem<Controller>::dynamicsystem(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN, Controller& C)
{
	_controllerClass = C;
	initializestate(initialState_IN, initialInput_IN, time_IN, sLen_IN, iLen_IN);
}

template<class Controller>
void dynamicsystem<Controller>::initializestate(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN)
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

template<class Controller>
void dynamicsystem<Controller>::SetEstimator(const std::function<Matrix(Matrix,Matrix,double)>& newEstimator_IN)
{
	_estimator = newEstimator_IN;
}

template<class Controller>
void dynamicsystem<Controller>::SetController(const std::function<Matrix(Matrix,Matrix,double, Controller&)>& newController_IN)
{
	_controllerCB = newController_IN;
}

template<class Controller>
void dynamicsystem<Controller>::SetParameters()
{
	std::cout << "SetParameters() needs to be defined at derived class.\n"; 
}

template<class Controller>
void dynamicsystem<Controller>::Simulate()
{
	try{
		if(_controllerCB != NULL)
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
				_state = _estimator(_state, _input, currentTime);
				_input = _controllerCB(_state, _input, currentTime, _controllerClass);
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

template<class Controller>
void dynamicsystem<Controller>::Reset(Matrix resetState_IN, Matrix resetInput_IN)
{
	_stateVector.clear();
	_inputVector.clear();
	_timeVector.clear();

	_state = resetState_IN;
	_input = resetInput_IN;	
}

template<class Controller>
void dynamicsystem<Controller>::ExportCSV(char const* s)
{
	try{
		if(_timeVector.size() > 0)
		{
			std::ofstream output_file (s);
			for(size_t r = 0; r<_timeVector.size(); r++)
			{
				output_file << _timeVector[r] << ", ";
				for(unsigned i = 0; i<_sLen; i++)
				{
					output_file << _stateVector[r](i,0) << ", ";
				}
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

template<class Controller>
Matrix dynamicsystem<Controller>::integrator(Matrix state_IN, Matrix input_IN)	
{
	Matrix s_dot = diff(state_IN, input_IN);
	return _state + s_dot * _dt;
}	

#endif