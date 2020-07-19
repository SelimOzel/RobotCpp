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
	// Constructor(s)
	dynamicsystem(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN);
	dynamicsystem(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN, Controller& C);
	dynamicsystem(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN, Estimator& E);
	dynamicsystem(Matrix initialState_IN, Matrix initialInput_IN, std::vector<double> time_IN, int sLen_IN, int iLen_IN, Controller& C, Estimator& E);

	// Change the dynamic system controller
	void SetController(const std::function<Matrix(Matrix,Matrix,double,Controller&)>& newController_IN);

	// Change dynamic system estimator
	void SetEstimator(const std::function<Matrix(Matrix,Matrix,double,Estimator&)>& newEstimator_IN);

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

	// Simple callbacks for estimator/controller
	std::function<Matrix(Matrix, Matrix, double, Estimator&)> _estimatorCB = NULL;				
	std::function<Matrix(Matrix, Matrix, double, Controller&)> _controllerCB = NULL;	

	// Advanced classes for estimator/controller
	Controller _controllerClass;
	Estimator _estimatorClass;

	// States & Inputs: current values
	Matrix _state;
	Matrix _stateEstimated;
	Matrix _input;

	size_t _sLen = 0;		// State vector dimension
	size_t _iLen = 0;		// Input vector dimension

	// Integration
	double _dt = 0.0;		// Integrator delta [s]
	double _ft = 0.0;		// Final time [s]

	// Output 
	std::vector<Matrix> _stateVector;
	std::vector<Matrix> _stateEstimatedVector;
	std::vector<Matrix> _inputVector;
	std::vector<double> _timeVector;
};

#include "dynamicsystem.cpp"
#endif