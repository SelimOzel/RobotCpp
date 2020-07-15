/*
██████╗  ██████╗ ██████╗  ██████╗ ████████╗ ██████╗██████╗ ██████╗ 
██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗
██████╔╝██║   ██║██████╔╝██║   ██║   ██║   ██║     ██████╔╝██████╔╝
██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   ██║     ██╔═══╝ ██╔═══╝ 
██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   ╚██████╗██║     ██║     
╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝    ╚═════╝╚═╝     ╚═╝                                                                   
*/

#ifndef __LINEARALGEBRA_CPP
#define __LINEARALGEBRA_CPP

#include "linearalgebra.h"

Matrix::Matrix(){}

Matrix::Matrix(unsigned n, double v)
{
	reshape(n, n, 0.0);
	for(unsigned i = 0; i<n; i++)
	{
		_m[i][i] = n;
	}
}

Matrix::Matrix(const std::vector<double>& m)
{
	reshape(m.size(), 1, 0.0);
	for(unsigned r = 0; r < _nr; r++)
	{
		_m[r][0] = m[r];
	}		
}

Matrix::Matrix(const std::vector<std::vector<double>>& m)
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
}

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

Matrix Matrix::operator=(const std::vector<double>& m)
{
	reshape(m.size(), 1, 0.0);
	for(unsigned r = 0; r < _nr; r++)
	{
		_m[r][0] = m[r];
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
			if(m(r,c) != _m[r][c]) return false;
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
	if(r<_nr)
	{
		return _m[r]; 
	}
	else
	{
		throw "Element Access Error: index out of bounds\n";
	}
}

double Matrix::operator() (unsigned r, unsigned c) const
{
	if(r<_nr && c<_nc)
	{
		return _m[r][c]; 
	}
	else
	{
		throw "Element Access Error: index out of bounds\n";
	}	
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

void Matrix::Set(unsigned r, unsigned c, double v)
{
	if(r < _nr && c < _nc)
	{
		_m[r][c] = v;
	}
	else
	{
		throw std::runtime_error("Set Error: index out of bounds");
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

// Obtained from geeks-for-geeks: https://www.geeksforgeeks.org/determinant-of-a-matrix/
double Matrix::Det(unsigned n) const
{
	unsigned nr = _m.size();
	unsigned nc = _m[0].size();

	if(nr == nc)
	{
		//  Base case : if matrix contains single element 
		double D = 0; // Initialize result 
		if (n == 1) return _m[0][0]; 

		//std::vector<std::vector<double>> temp(n, std::vector<double>(n)); // To store cofactors 
		Matrix temp(n,n,0.0); // To store cofactors 

		int sign = 1;  // To store sign multiplier 

		// Iterate for each element of first row 
		for (unsigned f = 0; f < n; f++) 
		{
			// Getting Cofactor of mat[0][f] 
			cofactor(temp, 0, f, n); 
			D += sign * _m[0][f] * temp.Det(n - 1); 
			// terms are to be added with alternate sign 
			sign = -sign; 
		}

		return D;	
	}
	else
	{
		throw std::runtime_error("Determinant error: not square\n");
	}
}

std::vector<unsigned> Matrix::Size() const
{
	return {_nr, _nc};
}

Matrix Matrix::Inv() const
{
	unsigned nr = _m.size();
	unsigned nc = _m[0].size();

	if(nr == nc)
	{	
	    // Find determinant of A[][] 
	    double determinant = Det(nr); 
	    if (determinant == 0) 
	    { 
	        throw std::runtime_error("Inverse error: determinant must be non-zero\n"); 
	    } 
	  
	    // Inverse
	    Matrix inverse(nr,nr,0.0);

	    // Find adjoint 
	    Matrix adj(nr, nr, 0.0);
	    adjoint(adj); 

	    // Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
	    for (unsigned i=0; i<nr; i++) 
	        for (unsigned j=0; j<nr; j++) 
	            inverse.Set(i,j,adj(i,j)/determinant); 
	  
	    return inverse; 
    }	
	else
	{
		throw std::runtime_error("Inverse error: not square\n");
	}    
}

void Matrix::Size(const Matrix& m)
{
	std::cout<< "Rows: " << m.Size()[0] << " Columns:" << m.Size()[1] << "\n";
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

// Obtained from geeks-for-geeks: https://www.geeksforgeeks.org/determinant-of-a-matrix/
void Matrix::cofactor(Matrix& temp, unsigned p, unsigned q, unsigned n) const
{
    unsigned i = 0, j = 0; 
  
    // Looping for each element of the matrix 
    for (unsigned row = 0; row < n; row++) 
    { 
        for (unsigned col = 0; col < n; col++) 
        { 
            //  Copying into temporary matrix only those element 
            //  which are not in given row and column 
            if (row != p && col != q) 
            { 
            	temp.Set(i,j++,_m[row][col]);
                //temp[i][j++] = _m[row][col]; 
  
                // Row is filled, so increase row index and 
                // reset col index 
                if (j == n - 1) 
                { 
                    j = 0; 
                    i++; 
                } 
            } 
        } 
    } 	
}

// Obtained from geeks-for-geeks: https://www.geeksforgeeks.org/adjoint-inverse-matrix/
void Matrix::adjoint(Matrix& adj) const
{ 
	unsigned N = _nr;
    if (N == 1) 
    { 
        adj.Set(0,0,1); 
        return; 
    } 
  
    // temp is used to store cofactors of A[][] 
    int sign = 1;
    //std::vector<std::vector<double>> temp(N, std::vector<double>(N)); // To store cofactors 
  	Matrix temp(N, N, 0.0);

    for (int i=0; i<N; i++) 
    { 
        for (int j=0; j<N; j++) 
        { 
            // Get cofactor of A[i][j] 
            cofactor(temp, i, j, N); 

            // sign of adj[j][i] positive if sum of row 
            // and column indexes is even. 
            sign = ((i+j)%2==0)? 1: -1; 
  
            // Interchanging rows and columns to get the 
            // transpose of the cofactor matrix 
            adj.Set(j,i, (sign)*(temp.Det(N-1)));
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