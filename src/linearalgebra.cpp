#ifndef __LINEARALGEBRA_CPP
#define __LINEARALGEBRA_CPP

#include "linearalgebra.h"

Matrix::Matrix(){}
Matrix::Matrix(unsigned nr, unsigned nc, double n)
{
	reshape(nr, nc, n);
}

Matrix Matrix::operator=(const Matrix& m)
{
	reshape(m._nr, m._nc, 0.0);
	for(unsigned r = 0; r < _nr; r++)
	{
		for(unsigned c = 0; c < _nc; c++)
		{
			_m[r][c] = m._m[r][c];
		}
	}
	return *this;				
}

Matrix Matrix::operator=(const std::vector<std::vector<double>>& m)
{
	reshape(m.size(), m[0].size(), 0.0);
	for(unsigned r = 0; r < _nr; r++)
	{
		if(m[r].size() != m[0].size()) throw std::runtime_error("Assignment Error: Uneven row lengths");
		for(unsigned c = 0; c < _nc; c++)
		{
			_m[r][c] = m[r][c];
		}
	}
	return *this;				
}

Matrix Matrix::operator+(const Matrix& b) const
{
	if(b._nr == _nr && b._nc == _nc)
	{		
		Matrix result(_nr, _nc, 0.0);
		for(unsigned r = 0; r < _nr; r++)
		{
			for(unsigned c = 0; c < _nc; c++)
			{
				result._m[r][c] = _m[r][c] + b._m[r][c];
			}
		}
		return result;
	}
	else
	{
		throw std::runtime_error("Addition Error: size");
	}		
}	

Matrix Matrix::operator+=(const Matrix& b) 
{
	if(b._nr == _nr && b._nc == _nc)
	{
		Matrix result = *this + b;
		*this = result;
		return *this;			
	}
	else
	{
		throw std::runtime_error("Addition Error: size");
	}
}	

Matrix Matrix::operator*(const Matrix& b) const
{
	unsigned br = b._nr;
	unsigned bc = b._nc;
	Matrix result(_nr, bc, 0.0);

	// Verify mXn * nXp condition
	if(_nc == br)
	{
		for (unsigned r = 0; r<_nr; r++) 
		{
			for (unsigned c = 0; c<bc; c++) 
			{
				for (unsigned k = 0; k<_nc; k++) 
				{
					result._m[r][c] += _m[r][k] * b._m[k][c];
				}
			}
		}
		return result;
	}
	else
	{
		throw std::runtime_error("Multiplication Error: size");
	}
}

Matrix Matrix::operator*=(const Matrix& b) 
{
	unsigned br = b._nr;
	// Verify mXn * nXp condition
	if(_nc == br)
	{
		Matrix result = *this * b;
		*this = result;
		return *this;
	}
	else
	{
		throw std::runtime_error("Multiplication Error: size");
	}
}

Matrix Matrix::operator-(const Matrix& b) const
{
	if(b._nr == _nr && b._nc == _nc)
	{		
		Matrix result(_nr, _nc, 0.0);
		for(unsigned r = 0; r < _nr; r++)
		{
			for(unsigned c = 0; c < _nc; c++)
			{
				result._m[r][c] = _m[r][c] - b._m[r][c];
			}
		}
		return result;
	}
	else
	{
		throw std::runtime_error("Subtraction Error: size");
	}		
}

Matrix Matrix::operator-=(const Matrix& b) 
{
	if(b._nr == _nr && b._nc == _nc)
	{
		Matrix result = *this - b;
		*this = result;
		return *this;	
	}
	else
	{
		throw std::runtime_error("Subtraction Error: size");
	}
}

Matrix Matrix::operator+(double s) const
{
	Matrix result(_nr, _nc, 0.0);
	for(unsigned r = 0; r < _nr; r++)
	{
		for(unsigned c = 0; c < _nc; c++)
		{
			result._m[r][c] = _m[r][c] + s;
		}
	}
	return result;
}

Matrix Matrix::operator+=(const double s) 
{
	Matrix result = *this + s;
	*this = result;
	return *this;	
}	

Matrix Matrix::operator*(double s) const
{
	Matrix result(_nr, _nc, 0.0);
	for(unsigned r = 0; r < _nr; r++)
	{
		for(unsigned c = 0; c < _nc; c++)
		{
			result._m[r][c] = _m[r][c] * s;
		}
	}
	return result;
}

Matrix Matrix::operator*=(const double s) 
{
	Matrix result = *this * s;
	*this = result;
	return *this;	
}	

Matrix Matrix::operator-(double s) const
{
	Matrix result(_nr, _nc, 0.0);
	for(unsigned r = 0; r < _nr; r++)
	{
		for(unsigned c = 0; c < _nc; c++)
		{
			result._m[r][c] = _m[r][c] - s;
		}
	}
	return result;
}

Matrix Matrix::operator-=(const double s) 
{
	Matrix result = *this - s;
	*this = result;
	return *this;	
}

Matrix Matrix::operator/(double s) const
{
	Matrix result(_nr, _nc, 0.0);
	for(unsigned r = 0; r < _nr; r++)
	{
		for(unsigned c = 0; c < _nc; c++)
		{
			result._m[r][c] = _m[r][c] / s;
		}
	}
	return result;
}

Matrix Matrix::operator/=(const double s) 
{
	Matrix result = *this / s;
	*this = result;
	return *this;	
}	

Matrix Matrix::operator[] (unsigned i) const
{
	// Special case for row vectors
	if(_nr == 1)
	{
		if(i<_nc)
		{
			Matrix d(1,1,_m[0][i]);
			return d;
		}
		else
		{
			throw std::runtime_error("Column index out of bounds");
		}			
	}

	// Get row
	if(i<_nr)
	{
		Matrix result(1, _nc, 0.0);
		for(int c = 0; c<_nc; c++)
		{
			result._m[0][c] = _m[i][c];
		}

		return result;
	}
	else
	{
		throw std::runtime_error("Row index out of bounds");
	}
}

// Checks if matrices are equal
bool Matrix::operator== (const Matrix &m) const
{
	if(_nr != m.Size()[0]) return false;
	if(_nc != m.Size()[1]) return false;
	for(unsigned r = 0; r < _nr; r++)
	{
		for(unsigned c = 0; c < _nc; c++)
		{
			if(m(r,c) != _m[r][c]) false;
		}
	}
	return true;
}

std::vector<std::vector<double>> Matrix::operator() () const
{
	return _m;
}

std::vector<double> Matrix::operator() (unsigned r) const
{
	return _m[r];
}

double Matrix::operator() (unsigned r, unsigned c) const
{
	return _m[r][c];
}

double Matrix::Sum() const
{
	double result = 0.0;
	for(unsigned r = 0; r < _nr; r++)
	{
		for(unsigned c = 0; c < _nc; c++)
		{
			result += _m[r][c];
		}
	}
	return result;		
}

double Matrix::Sum(unsigned r) const
{
	if(r < _nr)
	{
		double result = 0.0;
		for(unsigned c = 0; c < _nc; c++)
		{
			result += _m[r][c];
		}
		return result;	
	}
	else
	{
		throw std::runtime_error("Sum Row Error: index out of bounds");
	}
}

Matrix Matrix::T() const
{
	std::vector<std::vector<double>> result(_nc, std::vector<double>(_nr));
	Matrix resultM(_nc, _nr, 0.0);
	for (unsigned r=0; r<resultM.Size()[0]; r++) 
	{
		for (unsigned c=0; c<resultM.Size()[1]; c++) 
		{
			resultM._m[r][c] = _m[c][r];
		}
	}
	return resultM;
}

std::vector<unsigned> Matrix::Size() const
{
	return {_nr, _nc};
}

void Matrix::Print(const Matrix& m)
{
	for(unsigned r = 0; r < m._nr; r++)
	{
		for(unsigned c = 0; c < m._nc; c++)
		{
			std::cout << m._m[r][c];
			if(c == m._nc - 1) std::cout << "\n";
			else std::cout << ", ";
		}
	}
}	

void Matrix::reshape(unsigned r_IN, unsigned c_IN, double n_IN)
{
	if(r_IN != 0 && c_IN != 0)
	{
		_m.resize(r_IN); 
		for (unsigned r=0; r<_m.size(); r++) 
		{
			_m[r].resize(c_IN,n_IN);
		}	
		_nr = r_IN;
		_nc = c_IN;
	}
	else
	{
		throw std::runtime_error("Dimensions can't be zero");
	}
}

#endif