/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/
#ifndef __FILTERS_CPP
#define __FILTERS_CPP

#include "filters.h"

KalmanFilter::KalmanFilter(const Matrix& F_IN, const Matrix& H_IN, const Matrix& B_IN)
{
	_F = F_IN; // F is NxN: State transition matrix. N is number of kalman states.
	_H = H_IN; // H is MxN: Sensor matrix. M is number of measured variables.
	_B = B_IN; // B is NxK: Input matrix. K is number of external motion/forces.
}

// x is Nx1 vector: Prediction from previous filter output
// P is NxN matrix: Correlation from previous filter output
// z is Nx1 vector: Sensor measurement at current cycle.
// u is Kx1 vector: K is number of external motion/forces.
std::vector<Matrix> KalmanFilter::Filter(Matrix& x, Matrix& P, Matrix& z, Matrix& u)
{
	// Variables
	Matrix x_prediction;
	Matrix P_prediction;
	Matrix y;
	Matrix s;
	Matrix K;
	Matrix x_next;
	Matrix P_next;
	std::vector<Matrix> result(2);

	// Prediction Update
	x_prediction = _F*x + _B*u;
	P_prediction = _F*P*_F.T(); // Q matrix can be added here for external noise.

	// Measurement Update
	y = z - _H*x_prediction;
	s = _H*P_prediction*_H.T(); // R matrix here can be added here for sensor noise
	//K = P_prediction*_H.T()

	// Kalman Filter Update


	// Fill the result
	result[0] = x_next;
	result[1] = P_next;
	return result;
}

#endif