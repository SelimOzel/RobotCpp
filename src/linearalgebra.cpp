// I/O Stream
#include <iostream>

// Vector
#include <vector>

// Lightweight linear algebra class to accompany RobotCpp

// Matrix operations
class Matrix
{
public:
	// Constructor. 
	Matrix(int nr, int nc)
	{
		if(nr >= 0 && nc >= 0)
		{
			_nr = nr;
			_nc = nc;

			// All matrices shall be initialized with 1.0. 
			for(int r = 0; r < nr; r++)
			{
				std::vector<double> row;
				for(int c = 0; c < nc; c++)
				{
					row.push_back(1.0);
				}
				_m.push_back(row);
				row.clear();
			}
		}
	}

	// Writes matrix to standard output
	static void Print(const Matrix& m)
	{
		for(int r = 0; r < m._nr; r++)
		{
			for(int c = 0; c < m._nc; c++)
			{
				std::cout << m._m[r][c];
				if(c == m._nc - 1) std::cout << "\n";
				else std::cout << ", ";
			}
		}
	}

	Matrix operator= (const std::vector<std::vector<double>>& m)
	{
		std::cout<<"hi";
		Matrix m_new(m.size(), m[0].size());
		for(int r = 0; r < _nr; r++)
		{
			for(int c = 0; c < _nc; c++)
			{std::cout<<m[r][c];
				m_new._m[r][c] = m[r][c];
			}
		}
		return m_new;				
	}

	Matrix operator[] (int r) 
	{
		return Matrix();
	}

	// Matrix sum: a + b 
	Matrix operator+(const Matrix& b)
	{
		Matrix a(b._nr, b._nc);
		for(int r = 0; r < _nr; r++)
		{
			for(int c = 0; c < _nc; c++)
			{
				a._m[r][c] += b._m[r][c];
			}
		}
		return a;
	}

private:
	Matrix()
	{
	}

	int _nr = 0;
	int _nc = 0;

	std::vector<std::vector<double>> _m;
};

int main()
{
	std::cout<< "Linear Algebra Tests:\n\n";
	std::cout<< "Starting matrix tests...\n\n";

	Matrix X1(0,0);
	Matrix X2(-1,0);
	std::cout<< "Constructer tests finished.\n\n";
	
	Matrix A(2,2);
	std::vector<std::vector<double>> m = {{2,3},{4,5}};
	A = m;
	Matrix B(2,2);
	Matrix C(1,1);

	Matrix::Print(X1);
	std::cout<< "Prints nothing.\n\n";
	Matrix::Print(A);
	std::cout<< "2x2.\n\n";
	Matrix A2(2,3);
	Matrix A3(3,2);
	Matrix::Print(A2);
	std::cout<< "2x3.\n\n";
	Matrix::Print(A3);	
	std::cout<< "3x2.\n\n";

	
	std::cout<< "A+A2 = "; 
	C = A + A2;
	std::cout<< "A+A3 = "; 
	C = A + A3;
	C = A + B;
	Matrix::Print(C);
	std::cout<< "+overload tests finished.\n\n";


	return 1;
}