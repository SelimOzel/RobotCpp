// I/O Stream
#include <iostream>

// Vector
#include <vector>

// Unit testing
#include <cassert>

// Lightweight linear algebra class to accompany RobotCpp

// Matrix
class Matrix
{
public:
	// Constructor(s)
	Matrix(){}
	Matrix(unsigned nr, unsigned nc, double n)
	{
		reshape(nr, nc, n);
	}

	// Overloaded assignment for matrix inputs
	Matrix operator= (const Matrix& m)
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

	// Convenience for setting via inline 2D vectors.
	Matrix operator= (const std::vector<std::vector<double>>& m)
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

	// Matrix addition
	Matrix operator+(const Matrix& b) const
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

	// Matrix cum. addition
	Matrix operator+= (const Matrix& b) 
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

	// Matrix multiplication
	Matrix operator*(const Matrix& b) const
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

	// Matrix cum. multiplication
	Matrix operator*=(const Matrix& b) 
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

	// Matrix subtraction
	Matrix operator-(const Matrix& b) const
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

	// Matrix cum. subtraction
	Matrix operator-= (const Matrix& b) 
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

	// Scalar sum
	Matrix operator+(double s) const
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

	// Scalar cum. sum
	Matrix operator+= (const double s) 
	{
		Matrix result = *this + s;
		*this = result;
		return *this;	
	}	

	// Scalar multiply
	Matrix operator*(double s) const
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
	
	// Scalar cum. multiply
	Matrix operator*= (const double s) 
	{
		Matrix result = *this * s;
		*this = result;
		return *this;	
	}		

	// Scalar subtraction
	Matrix operator-(double s) const
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

	// Scalar cum. subtraction
	Matrix operator-= (const double s) 
	{
		Matrix result = *this - s;
		*this = result;
		return *this;	
	}

	// Scalar division
	Matrix operator/(double s) const
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

	// Scalar cum. division
	Matrix operator/= (const double s) 
	{
		Matrix result = *this / s;
		*this = result;
		return *this;	
	}	

	// Single [r] accesses a row, double [r][c] accesses an element 
	// IMPORTANT: This overload returns Matrix; not, vector, not double ...
	Matrix operator[] (unsigned i) const
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
	bool operator== (const Matrix &m) const
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

	// Returns row at (r) as std::vector<std::vector<double>>
	std::vector<std::vector<double>> operator() () const
	{
		return _m;
	}

	// Returns row at (r) as std::vector<double>
	std::vector<double> operator() (unsigned r) const
	{
		return _m[r];
	}

	// Returns value at (r,c) as double
	double operator() (unsigned r, unsigned c) const
	{
		return _m[r][c];
	}

	// Sums all elements
	double Sum() const
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

	// Sums all elements in row r.
	double Sum(unsigned r) const
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

	// Return transpose of nXm matrix. Result is mXn.
	Matrix T() const
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

	// Returns size as [r, c]
	std::vector<unsigned> Size() const
	{
		return {_nr, _nc};
	}

	// Writes matrix to standard output
	static void Print(const Matrix& m)
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

private:
	void reshape(unsigned r_IN, unsigned c_IN, double n_IN)
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

	unsigned _nr = 0;
	unsigned _nc = 0;

	std::vector<std::vector<double>> _m;
};

int main()
{
	std::cout<< "Linear Algebra Tests:\n\n";
	std::cout<< "Starting matrix tests...\n\n";

	// Assignment and creator operations
	Matrix A(2, 2, 1.0); // Initialize all elements to 1.
	A = {{2,3},{3,2}}; // Set elements
	assert(A.Size()[0] == 2 && A.Size()[1] == 2);

	Matrix::Print(A);
	std::cout<<"\n";

	A = {{7,8},{8,7},{8,9}}; // Set elements and reshape
	assert(A.Size()[0] == 3 && A.Size()[1] == 2);

	Matrix B; // A matrix can be created without initializing
	B = {{3,2},{2,3},{2,1}}; // Set elements
	assert(B.Size()[0] == 3 && B.Size()[1] == 2);

	Matrix C;
	C = B + A; // Matrix addition
	assert(C.Size()[0] == 3 && C.Size()[1] == 2);
	assert(C.Sum() == 60);
	assert(C.Sum(0) == 20);
	assert(C.Sum(1) == 20);
	assert(C.Sum(2) == 20);

	Matrix::Print(C);
	std::cout<<"\n";

	A = C; // Matrix assignment	
	assert(A.Size()[0] == 3 && A.Size()[1] == 2);

	// Addition-Subtraction-Multiplication-Division(scalar)
	B = {{3,2},{2,3}};
	A = {{2,3},{3,2}}; // Set elements
	assert(A.Size()[0] == B.Size()[0] && A.Size()[1] == B.Size()[1]);
	
	A += B; // cumulative matrix addition
	A += 1;	// cumulative scalar addition
	assert(A.Sum(0) + A.Sum(1) == A.Sum() && A.Sum() == 24);

	A -= 1; // cumulative matrix subtraction
	A = A - B; // matrix subtraction
	A = A - 1; // cumulative scalar subtraction
	assert(A.Sum(0) + A.Sum(1) == A.Sum() && A.Sum() == 6);

	A = {{3,2,1},{1,2,3},{0,1,2}}; // 3x3
	B = {{3,2},{1,4},{1,5}}; // 3x2
	A *= B;
	B = A.T();
	assert(A(0,0) + A(0,1) == A.Sum(0));
	assert(A(1,0) + A(1,1) == A.Sum(1));
	assert(A(2,0) + A(2,1) == A.Sum(2)); // row sum verifications
	assert(A.Sum() == A.Sum(0)+A.Sum(1)+A.Sum(2)); // row sum equals all element sum	
	assert(B.T() == A);

	Matrix::Print(A);
	std::cout<<"\n";	

	Matrix::Print(B[0].T()); // {12,8,3}
	std::cout<<"\n";

	C = B[0].T(); // Convert a row to column vector
	A = {{{2,0,1}},{{0,2,1}}}; 
	assert(A*B[0].T() == A*C);

	std::cout<< "n: " << A.Size()[0] << " m:" << A.Size()[1] << "\n";
	std::cout<< "n: " << C.Size()[0] << " m:" << C.Size()[1] << "\n\n";
	Matrix::Print(A*B[0].T()); // Convert B to column vector and multiply with 2x3 matrix. 
	
	std::cout<<"\n";


	return 1;
}