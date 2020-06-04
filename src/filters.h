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
	// Constructor
	KalmanFilter(const Matrix& F_IN, const Matrix& H_IN, const Matrix& B_IN);
	
	// Filter
	std::vector<Matrix> Filter(Matrix& x, Matrix& P, Matrix& z, Matrix& u);

private:
	Matrix _F; // State transition matrix
	Matrix _H; // Sensor matrix
	Matrix _B; // Input matrix
};

#include "filters.cpp"
#endif