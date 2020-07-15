/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/
#ifndef __FILTERS_H
#define __FILTERS_H

#include "linearalgebra.h"

class KalmanFilter
{
public:
	// Constructor(s)
	KalmanFilter();
	KalmanFilter(const Matrix& F_IN, const Matrix& H_IN, const Matrix& B_IN, const Matrix& x_init, const Matrix& P_init);
	
	// Filter
	Matrix Filter(Matrix& z, Matrix& u);

	// Initialize internal filter matrices
	void Initialize(const Matrix& F_IN, const Matrix& H_IN, const Matrix& B_IN, const Matrix& x_init, const Matrix& P_init);

private:
	Matrix _F; // State transition matrix
	Matrix _H; // Sensor matrix
	Matrix _B; // Input matrix

	Matrix _x_prediction; // Keeps track of state prediction
	Matrix _P_prediction; // Keeps track of covariance matrix
};

#include "filters.cpp"
#endif