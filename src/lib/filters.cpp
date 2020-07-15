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

KalmanFilter::KalmanFilter(){}

KalmanFilter::KalmanFilter(const Matrix& F_IN, const Matrix& H_IN, const Matrix& B_IN, const Matrix& x_init, const Matrix& P_init)
{
	Initialize(F_IN, H_IN, B_IN, x_init, P_init);
}

// x is Nx1 vector: Prediction from previous filter output
// P is NxN matrix: Correlation from previous filter output
// z is Nx1 vector: Sensor measurement at current cycle.
// u is Kx1 vector: K is number of external motion/forces.
Matrix KalmanFilter::Filter(Matrix& z, Matrix& u)
{
	// Variables
	Matrix y;
	Matrix s;
	Matrix K;
	Matrix x_next;
	Matrix P_next;
	std::vector<Matrix> result(2);

	// Prediction Update
	_x_prediction = _F*_x_prediction + _B*u;
	_P_prediction = _F*_P_prediction*_F.T(); // Q matrix can be added here for external noise.

	// Measurement Update
	y = z - _H*_x_prediction;
	s = _H*_P_prediction*_H.T(); // R matrix here can be added here for sensor noise
	K = _P_prediction*_H.T()*s.Inv();

	// Kalman Filter Update
	x_next = _x_prediction + K*y;
	P_next = _P_prediction - K*_H*_P_prediction;

	// Update stored filter values
	_x_prediction = x_next;
	_P_prediction = P_next;

	return x_next;
}

void KalmanFilter::Initialize(const Matrix& F_IN, const Matrix& H_IN, const Matrix& B_IN, const Matrix& x_init, const Matrix& P_init)
{
	_F = F_IN; // F is NxN: State transition matrix. N is number of kalman states.
	_H = H_IN; // H is MxN: Sensor matrix. M is number of measured variables.
	_B = B_IN; // B is NxK: Input matrix. K is number of external motion/forces.

	_x_prediction = x_init;
	_P_prediction = P_init;	
}

#endif