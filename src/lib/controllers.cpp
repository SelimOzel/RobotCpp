/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/
#ifndef __CONTROLLERS_CPP
#define __CONTROLLERS_CPP

#include "controllers.h"

PID::PID(){}
PID::PID(double kp, double kd, double ki, int integrator_length)
{
	setKp(kp);
	setKd(kd);
	setKi(ki);
	setIntegrator(integrator_length);
}

void PID::setKp(double kp)
{
	_kp = kp;
}

void PID::setKd(double kd)
{
	_kd = kd;
}

void PID::setKi(double ki)
{
	_ki = ki;
}

void PID::setIntegrator(int integrator_length)
{
	clear();
	_integrator_length = integrator_length;
}

double PID::compute(double e, double e_dot)
{
	if(_integrator.size() >= _integrator_length) 
	{
		_ei -= _integrator.front();
		_integrator.pop();
	}
	_ei += e;
	_integrator.push(e);

	return _kp*e + _kd*e_dot + _ki*_ei;
}

void PID::clear()
{
	std::queue<double> empty;
	std::swap( _integrator, empty );
	_ei = 0.0;
}

#endif