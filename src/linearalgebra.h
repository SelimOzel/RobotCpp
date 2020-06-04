/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/
#ifndef __LINEARALGEBRA_H
#define __LINEARALGEBRA_H

#include <iostream> // I/O Stream
#include <vector> // Vector
#include <cassert> // Unit testing

// Lightweight linear algebra class to accompany RobotCpp
// Matrix
class Matrix
{
public:
	// Constructor(s)
	Matrix();
	Matrix(const std::vector<double>& m);
	Matrix(const std::vector<std::vector<double>>& m);
	Matrix(unsigned nr, unsigned nc, double n);

	// Overloaded assignment for matrix inputs
	Matrix operator=(const Matrix& m);

	// Convenience for setting via inline 1D column vectors.
	Matrix operator=(const std::vector<double>& m);

	// Convenience for setting via inline 2D vectors.
	Matrix operator=(const std::vector<std::vector<double>>& m);

	// Matrix addition
	Matrix operator+(const Matrix& b) const;

	// Matrix cum. addition
	Matrix operator+=(const Matrix& b);

	// Matrix multiplication
	Matrix operator*(const Matrix& b) const;

	// Matrix cum. multiplication
	Matrix operator*=(const Matrix& b);

	// Matrix subtraction
	Matrix operator-(const Matrix& b) const;

	// Matrix cum. subtraction
	Matrix operator-= (const Matrix& b);	

	// Scalar sum
	Matrix operator+(double s) const;

	// Scalar cum. sum
	Matrix operator+= (const double s);	

	// Scalar multiply
	Matrix operator*(double s) const;
	
	// Scalar cum. multiply
	Matrix operator*= (const double s);	

	// Scalar subtraction
	Matrix operator-(double s) const;

	// Scalar cum. subtraction
	Matrix operator-= (const double s);

	// Scalar division
	Matrix operator/(double s) const;

	// Scalar cum. division
	Matrix operator/= (const double s);

	// Single [r] accesses a row, double [r][c] accesses an element 
	// IMPORTANT: This overload returns Matrix; not, vector, not double ...
	Matrix operator[] (unsigned i) const;

	// Checks if matrices are equal
	bool operator== (const Matrix &m) const;

	// Returns row at (r) as std::vector<std::vector<double>>
	std::vector<std::vector<double>> operator() () const;

	// Returns row at (r) as std::vector<double>
	std::vector<double> operator() (unsigned r) const;

	// Returns value at (r,c) as double
	double operator() (unsigned r, unsigned c) const;

	// Sums all elements
	double Sum() const;

	// Sums all elements in row r.
	double Sum(unsigned r) const;

	// Set element at index.
	void Set(unsigned r, unsigned c, double v);

	// Return transpose of nXm matrix. Result is mXn.
	Matrix T() const;

	// Return determinant
	double Det(unsigned n);

	// Returns size as [r, c]
	std::vector<unsigned> Size() const;

	// Utility function to print size to standard output
	static void Size(const Matrix& m);	

	// Writes matrix to standard output
	static void Print(const Matrix& m);	

private:
	void cofactor(Matrix& temp, unsigned p, unsigned q, unsigned n) const;
	void adjoint(Matrix& adj);
	void reshape(unsigned r_IN, unsigned c_IN, double n_IN);

	unsigned _nr = 0;
	unsigned _nc = 0;

	std::vector<std::vector<double>> _m;
};

#include "linearalgebra.cpp"
#endif