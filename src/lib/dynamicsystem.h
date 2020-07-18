/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

#ifndef __DYNAMICSYSTEM_H
#define __DYNAMICSYSTEM_H

#include "linearalgebra.h" // Lightweight linear algebra tools

#include <fstream> // Writing to csv file.
#include <cmath> // Math operations
#include <functional> // Used for passing controller to robot instances

class NOCONTROLLER {}; // Global definition for robotcpp. Use this in the model when a controller class is not needed.
class NOESTIMATOR {}; // Use this when no estimator is needed.

// Base class for all dynamic systems in RobotCpp
template<class Controller, class Estimator>
class dynamicsystem
{
public:
	// Constructor
	dynamicsystem(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN);
	dynamicsystem(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN, Controller& C);

	// Change the dynamic system controller
	void SetController(const std::function<Matrix(Matrix,Matrix,double,Controller&)>& newController_IN);

	// Change dynamic system estimator
	void SetEstimator(const std::function<Matrix(Matrix,Matrix,double)>& newEstimator_IN);

	// System parameters (i.e. mass, damping ...)
	virtual void SetParameters();

	// Simulation
	void Simulate();

	// Reset the system. Clears output vectors.
	void Reset(Matrix resetState_IN, Matrix resetInput_IN);

	// Export simulation csv file
	void ExportCSV(char const* s);

protected:
	// System Equations: Only diff is protected
	virtual Matrix diff(Matrix state_IN, Matrix input_IN) = 0; // Difference Equation. Defined in derived class.	

private:
	// Called by constructors
	void initializestate(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN);

	// System Equations: Private
	Matrix integrator(Matrix state_IN, Matrix input_IN);	// Simple integrator

	// Estimator: Defined in derived class or outside in the main program.
	std::function<Matrix(Matrix, Matrix, double)> _estimator = NULL;	

	// Controller: Defined in derived class or outside in the main program.				
	std::function<Matrix(Matrix, Matrix, double, Controller&)> _controllerCB = NULL;	

	// Advanced controller class
	Controller _controllerClass;

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

#include "dynamicsystem.cpp"
#endif