// I/O Stream
#include <iostream>

// Vector
#include <vector>

// Lightweight linear algebra class to accompany RobotCpp

// Matrix operations
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
			if(m[r].size() != m[0].size()) throw std::runtime_error("Assignment Error: Uneven row lengths.");
			for(unsigned c = 0; c < _nc; c++)
			{
				_m[r][c] = m[r][c];
			}
		}
		return *this;				
	}

	// Matrix sum: a + b 
	Matrix operator+(const Matrix& b)
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
			throw std::runtime_error("Addition Error: Matrix sizes don't match.");
		}		
	}

	// Scalar sum: a + 1.25
	Matrix operator+(double s)
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

	// Cumulative matrix sum.
	Matrix operator+= (const Matrix& b) 
	{
		if(b._nr == _nr && b._nc == _nc)
		{
			for(unsigned r = 0; r < _nr; r++)
			{
				for(unsigned c = 0; c < _nc; c++)
				{
					_m[r][c] += b._m[r][c];
				}
			}
			return *this;
		}
		else
		{
			throw std::runtime_error("Addition Error: Matrix sizes don't match.");
		}
	}	

	// Cumulative scalar sum.
	Matrix operator+= (const double s) 
	{
		for(unsigned r = 0; r < _nr; r++)
		{
			for(unsigned c = 0; c < _nc; c++)
			{
				_m[r][c] += s;
			}
		}
		return *this;
	}	

	// Matrix subtraction: a + b 
	Matrix operator-(const Matrix& b)
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
			throw std::runtime_error("Addition Error: Matrix sizes don't match.");
		}		
	}

	// Scalar subtraction: a + 1.25
	Matrix operator-(double s)
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

	// Cumulative matrix subtraction.
	Matrix operator-= (const Matrix& b) 
	{
		if(b._nr == _nr && b._nc == _nc)
		{
			for(unsigned r = 0; r < _nr; r++)
			{
				for(unsigned c = 0; c < _nc; c++)
				{
					_m[r][c] -= b._m[r][c];
				}
			}
			return *this;
		}
		else
		{
			throw std::runtime_error("Addition Error: Matrix sizes don't match.");
		}
	}	

	// Cumulative scalar subtraction.
	Matrix operator-= (const double s) 
	{
		for(unsigned r = 0; r < _nr; r++)
		{
			for(unsigned c = 0; c < _nc; c++)
			{
				_m[r][c] -= s;
			}
		}
		return *this;
	}

	Matrix operator[] (unsigned r) 
	{
		return *this;
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
		_m.resize(r_IN); 
		for (unsigned r=0; r<_m.size(); r++) 
		{
			_m[r].resize(c_IN,n_IN);
		}	
		_nr = _m.size();
		_nc = _m[0].size();
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
	Matrix::Print(A);
	std::cout<<"\n";

	A = {{7,8},{8,7},{8,9}}; // Set elements and reshape
	std::cout<<"\n";

	Matrix B; // A matrix can be created without initializing
	B = {{3,2},{2,3},{2,1}}; // Set elements
	Matrix C;
	C = B + A; // Matrix addition
	Matrix::Print(C);
	std::cout<<"\n";

	A = C; // Matrix assignment
	Matrix::Print(A);
	std::cout<<"\n";	

	// Addition-Subtraction
	B = {{3,2},{2,3}};
	A = {{2,3},{3,2}}; // Set elements
	A += B; // cumulative matrix addition
	A += 1;	// cumulative scalar addition
	Matrix::Print(A);
	std::cout<<"\n";

	A -= 1; // cumulative matrix subtraction
	A = A - B; // matrix subtraction
	A = A - 1; // cumulative scalar subtraction
	Matrix::Print(A);
	std::cout<<"\n";

	return 1;
}