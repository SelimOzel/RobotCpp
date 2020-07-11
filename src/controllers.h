/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/
#ifndef __CONTROLLERS_H
#define __CONTROLLERS_H

#include <queue>

class PID
{
public:
	// Constructor(s)
	PID();
	PID(double kp, double kd, double ki, int integrator_length);

	// Tuning
	void setKp(double kp);
	void setKd(double kd);
	void setKi(double ki);
	void setIntegrator(int integrator_length);

	// Utility
	double compute(double e, double e_dot);
	void clear(); // clears integrator

private:
	double _kp;
	double _kd;
	double _ki;
	int _integrator_length;	
	std::queue<double> _integrator;
	double _ei; // internal integrator sum
};

#include "controllers.cpp"
#endif