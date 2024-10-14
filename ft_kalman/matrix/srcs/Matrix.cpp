/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Matrix.cpp                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/05 19:06:08 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/15 19:20:39 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Matrix.hpp"

template <typename K>
Matrix<K>::Matrix(): _row(0), _column(0)
{
}

template <typename K>
Matrix<K>::Matrix(size_t row, size_t column): _row(row), _column(column)
{
	std::vector<std::vector<K>>	res(row, std::vector<K>(column));
	this->_matrix = res;
}

template <typename K>
Matrix<K>::Matrix(const std::vector<std::vector<K>> &matrix)
{
	this->_row = matrix.size();
	this->_column = matrix[0].size();
	
	for (size_t i = 1; i < this->_row; i++)
	{
		if (matrix[i].size() != this->_column)
		{
			std::string	msg = "error: invalid size of the input matrix";
			throw (msg);
		}
			
	}
	if (matrix[0].size() == 0 && this->_column == 0)
	{
		std::string	msg = "error: invalid size of the input matrix";
		throw (msg);
	}
	this->_matrix = matrix;
}

template <typename K>
Matrix<K>::Matrix(const Matrix<K> &matrix)
{
	*this = matrix;
}

template <typename K>
Matrix<K>& Matrix<K>::operator=(const Matrix<K> &matrix)
{
	if (this == &matrix)
		return (*this);
	this->_row = matrix._row;
	this->_column = matrix._column;
	this->_matrix = matrix._matrix;
	return (*this);
}

template <typename K>
Matrix<K>::~Matrix()
{
}

template <typename K>
size_t	Matrix<K>::getRowSize(void) const
{
	return (this->_row);
}

template <typename K>
size_t	Matrix<K>::getColumnSize(void) const
{
	return (this->_column);
}

template <typename K>
std::vector<std::vector<K>>	Matrix<K>::getMatrix(void) const
{
	return (this->_matrix);
}

template <typename K>
bool	Matrix<K>::isSquare(void) const
{
	if (this->_row == this->_column)
		return (true);
	return (false);
}

template <typename K>
void	Matrix<K>::printSize(void) const
{
	std::cout << this->_row << " by " << this->_column;
	if (isSquare())
		std::cout << " square";
	std::cout << " matrix" << std::endl;
}

template <typename K>
void	Matrix<K>::add(const Matrix<K> &matrix)
{
	if (this->_row != matrix.getRowSize() || getColumnSize() != matrix.getColumnSize())
	{
		std::string	msg = "error: cannot add matrices of different sizes";
		throw (msg);
	}
	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
			this->_matrix[r][c] += matrix.getMatrix()[r][c];
	}
}

template <typename K>
void	Matrix<K>::sub(const Matrix<K> &matrix)
{
	if (this->_row != matrix.getRowSize() || getColumnSize() != matrix.getColumnSize())
	{
		std::string	msg = "error: cannot subtract matrices of different sizes";
		throw (msg);
	}
	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
			this->_matrix[r][c] -= matrix.getMatrix()[r][c];
	}
}

template <typename K>
void	Matrix<K>::scale(const K scalar)
{
	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
			this->_matrix[r][c] *= scalar;
	}
}

template <typename K>
Vector<K>	Matrix<K>::mul_vec(const Vector<K> &vector) const
{
	if (this->_column != vector.getSize())
	{
		std::string	msg = "error: cannot multiply m by n matrices with non-n-demension vector";
		throw (msg);
	}

	std::vector<K>	res(this->_row);

	for (size_t m = 0; m < this->_row; m++)
	{
		for (size_t n = 0; n < this->_column; n++)
			res[m] += this->_matrix[m][n] * vector.getVector()[n];
	}
	return (Vector<K>(res));
}

template <typename K>
Matrix<K>	Matrix<K>::mul_mat(const Matrix<K> &matrix) const
{
	if (this->_column != matrix.getRowSize())
	{
		std::string	msg = "error: cannot multiply m by n matrix with non-n-by-p matrix";
		throw (msg);
	}

	std::vector<std::vector<K>>	res(this->_row, std::vector<K>(matrix.getColumnSize()));

	for (size_t m = 0; m < this->_row; m++)
	{
		for (size_t p = 0; p < matrix.getColumnSize(); p++)
		{
			for (size_t n = 0; n < this->_column; n++)
				res[m][p] += this->_matrix[m][n] * matrix.getMatrix()[n][p];
		}
	}
	return (Matrix<K>(res));
}

template <typename K>
K	Matrix<K>::trace(void) const
{
	if (isSquare() == false)
	{
		std::string	msg = "error: cannot define trace of non-square matrix";
		throw (msg);
	}

	K	res = K(0);

	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
		{
			if (r == c)
				res += this->_matrix[r][c];
		}
	}
	return (res);
}

template <typename K>
Matrix<K>	Matrix<K>::transpose(void) const
{
	std::vector<std::vector<K>>	res(this->_column, std::vector<K>(this->_row));

	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
			res[c][r] = this->_matrix[r][c];
	}
	return (Matrix<K>(res));
}

template <typename K>
Matrix<K>	Matrix<K>::conjugate(void) const
{
	std::vector<std::vector<K>>	res = this->_matrix;

	if constexpr (!std::is_arithmetic<K>::value)
	{
		for (size_t r = 0; r < this->_row; r++)
		{
			for (size_t c = 0; c < this->_row; c++)
				res[r][c] = res[r][c].conj();
		}
	}
	return (Matrix<K>(res));
}

template <typename K>
Matrix<K>	Matrix<K>::conjugateTranspose(void) const
{
	Matrix<K>	res(this->_matrix);
	
	return (res.conjugate().transpose());
}

/* row operation 1: swap two row
	ex) rowOperation_1(m, r1, r2) swaps m[r1] and m[r2] */
template <typename K>
void	Matrix<K>::rowOperation_1(std::vector<std::vector<K>> *m, size_t r1, size_t r2) const
{
	std::vector<K>	tmp = (*m)[r1];

	(*m)[r1] = (*m)[r2];
	(*m)[r2] = tmp;
}

/* row operation 2: multiply a row by a nonzero number
	ex) rowOperation_2(m, r, k): multiplies m[r] by k */
template <typename K>
void	Matrix<K>::rowOperation_2(std::vector<std::vector<K>> *m, size_t r, K k) const
{
	for (size_t c = 0; c < this->_column; c++)
		(*m)[r][c] *= k;
}

/* row operation 3: add a multiple of one row to another row
	ex) rowOperation_3(m, r1, k, r2) replaces m[r1] to m[r1] + k * m[r2] */
template <typename K>
void	Matrix<K>::rowOperation_3(std::vector<std::vector<K>> *m, size_t r1, K k, size_t r2) const
{
	for (size_t c = 0; c < this->_column; c++)
		(*m)[r1][c] += k * (*m)[r2][c];
}

template <typename K>
Matrix<K>	Matrix<K>::row_echelon(void) const
{
	std::vector<std::vector<K>>	res = this->_matrix;
	// size_t				pvt[this->_row] = {0, };
	std::vector<size_t>			pvt(this->_row, 0);

	std::function<void(size_t)>		setPivot = [&](size_t r)
	{
		for (size_t c = 0; c < this->_column; c++)
		{
			if constexpr (std::is_arithmetic<K>::value)
			{
				if (res[r][c] != 0 && K(-1) * K(EPSILON) < res[r][c] && res[r][c] < K(EPSILON))
					res[r][c] = K(0);
			}
			else
			{
				if (res[r][c].real() != 0 && -1 * EPSILON < res[r][c].real() && res[r][c].real() < EPSILON)
					res[r][c] = K(0, res[r][c].imag());
				if (res[r][c].imag() != 0 && -1 * EPSILON < res[r][c].imag() && res[r][c].imag() < EPSILON)
					res[r][c] = K(res[r][c].real(), 0);
			}
			
			if (res[r][c] != 0)
				break ;
			if ((c == 0 && this->_column > 1) || (c > 0 && res[r][c - 1] == 0))
				pvt[r]++;
		}
	};

	for (size_t r = 0; r < this->_row; r++)
		setPivot(r);

	/* row echelon form */
	for (size_t r = 0; r < this->_row; r++)
	{
		/* re-arrange rows */
		if (r < this->_row - 1 && pvt[r] > pvt[r + 1])
			rowOperation_1(&res, r, r + 1);
		
		/* gaussian elimination */
		if (r != 0 && pvt[r] <= pvt[r - 1])
		{
			for (size_t i = 0; i < r; i++)
			{
				if (pvt[i] != this->_column && res[i][pvt[i]] != 0)
					rowOperation_3(&res, r, -1 * res[r][pvt[i]] / res[i][pvt[i]], i);
				pvt[r] = 0;
				setPivot(r);
			}
		}
		if (pvt[r] != this->_column && res[r][pvt[r]] != 0)
			rowOperation_2(&res, r, 1 / res[r][pvt[r]]);
	}

	/* reduced row echelon form */
	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t i = 0; i < this->_row; i++)
		{
			if (i != r && pvt[r] != this->_column && res[r][pvt[r]] != 0)
				rowOperation_3(&res, i, -1 * res[i][pvt[r]] / res[r][pvt[r]], r);
		}
	}

	/* rounding res[r][c] to the nearest integer
		if they are close to being integers within a epsilon range */
	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
		{
			if constexpr (std::is_arithmetic<K>::value)
			{
				if (res[r][c] != 0 && -1 * EPSILON < res[r][c] - (int)(res[r][c]) && res[r][c] - (int)(res[r][c]) < EPSILON)
						res[r][c] = (int)res[r][c];
			}
			else
			{
				if (res[r][c].real() != 0 && -1 * EPSILON < res[r][c].real() - (int)(res[r][c].real()) && res[r][c].real() - (int)(res[r][c].real()) < EPSILON)
					res[r][c] = K((int)(res[r][c].real()), res[r][c].imag());
				if (res[r][c].imag() != 0 && -1 * EPSILON < res[r][c].imag() - (int)(res[r][c].imag()) && res[r][c].imag() - (int)(res[r][c].imag()) < EPSILON)
					res[r][c] = K(res[r][c].real(), (int)(res[r][c].imag()));
			}
		}
	}
	return (Matrix<K>(res));
}

template <typename K>
Matrix<K>	Matrix<K>::minor(size_t m, size_t n) const
{
	size_t	m_r = 0;
	size_t	m_c = 0;
	
	if (m >= this->_row || n >= this->_column)
	{
		std::string	msg = "error: invalid row or column";
		throw (msg);
	}

	std::vector<std::vector<K>>	res(this->_row - 1, std::vector<K>(this->_row - 1));

	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
		{
			if (r != m && c != n)
			{
				res[m_r][m_c] = this->_matrix[r][c];
				m_c++;
			}
		}
		if (m_c != 0)
			m_r++;
		m_c = 0;
	}
	return (Matrix<K>(res));
}

template <typename K>
K	Matrix<K>::determinant(void) const
{
	if (isSquare() == false)
	{
		std::string	msg = "error: cannot define determinant of non-square matrix";
		throw (msg);
	}

	if (getRowSize() == 1)
		return (this->_matrix[0][0]);
	
	K	res = K(0);

	for (size_t c = 0; c < this->_column; c++)
		res += (c % 2 == 0 ? 1: -1) * this->_matrix[0][c] * minor(0, c).determinant();
	return (res);
}

template <typename K>
Matrix<K>	Matrix<K>::cofactor(void) const
{
	if (getRowSize() == 1)
		return (Matrix<K>{{{this->_matrix[0][0]}}});

	std::vector<std::vector<K>>	res(this->_row, std::vector<K>(this->_column));

	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
			res[r][c] += ((r + c) % 2 == 0 ? 1 : -1) * minor(r, c).determinant();
	}
	return (Matrix<K>(res));
}

template <typename K>
Matrix<K>	Matrix<K>::inverse(void) const
{
	if (isSquare() == false)
	{
		std::string	msg = "error: cannot define inverse matrix of non-square matrix";
		throw (msg);
	}

	if (getRowSize() == 1)
		return (Matrix<K>{{{1 / this->_matrix[0][0]}}});

	K	det = determinant();

	if (det == 0)
	{
		std::string	msg = "error: singular matrix(determinant of the matrix is 0)";
		throw (msg);
	}

	Matrix<K>	res = cofactor().transpose();

	res.scale(1 / det);
	return (res);
}

template <typename K>
size_t	Matrix<K>::rank(void) const
{
	Matrix<K>	rref = row_echelon();
	size_t		res = this->_row;
	size_t		tmp = 0;

	for (size_t r = 0; r < this->_row; r++)
	{
		for (size_t c = 0; c < this->_column; c++)
		{
			if (rref.getMatrix()[r][c] != 0)
				break ;
			if ((c == 0 && this->_column > 1) || (c > 0 && rref.getMatrix()[r][c - 1] == 0))
				tmp++;
		}
		if (tmp == this->_column)
			res--;
		tmp = 0;
	}
	return (res);
}

template <typename K>
Matrix<K>	identity(size_t size)
{
	std::vector<std::vector<K>>	res(size, std::vector<K>(size));

	for (size_t r = 0; r < size; r++)
	{
		for (size_t c = 0; c < size; c++)
		{
			if (r == c)
			{
				if constexpr (std::is_arithmetic<K>::value)
					res[r][c] = 1;
				else
					res[r][c] = K(1, 0);
			}
		}
	}
	return (Matrix<K>(res));
}

template <typename K>
Matrix<K>	projection(K fov, K ratio, K near, K far)
{
	std::vector<std::vector<K>>	res(4, std::vector<K>(4));
	K				scale;

	if constexpr (std::is_arithmetic<K>::value)
		scale = 1 / std::tan(fov / 2 * M_PI / 180);
	else
	{
		if (fov.imag() != 0 || ratio.imag() != 0 || near.imag() != 0 || far.imag() != 0)
		{
			std::string	msg = "error: cannot calculate projection using parameter whose imaginary part is nonzero";
			throw (msg);
		}
		scale = K(1 / std::tan(fov.real() / 2 * M_PI / 180), 0);
	}
	res[0][0] = scale; // x coordinates of the projected point
	res[1][1] = scale / ratio; //  y coordinates of the projected point
	res[2][2] = -1 * far / (far - near); // remap z to [0,1]: 0(near)
	res[3][2] = -1 * far * near / (far - near); // remap z to [0,1]: 1(far)
	res[2][3] = -1; //set w = -z
	return (Matrix<K>(res));
}

template <typename K>
Matrix<K>	operator+(const Matrix<K> &l, const Matrix<K> &r)
{
	if (l.getRowSize() != r.getRowSize() || l.getColumnSize() != r.getColumnSize())
	{
		std::string	msg = "error: cannot add matrices of different sizes";
		throw (msg);
	}

	std::vector<std::vector<K>>	res(l.getRowSize(), std::vector<K>(l.getColumnSize()));
	for (size_t i = 0; i < l.getRowSize(); i++)
	{
		for (size_t j = 0; j < l.getColumnSize(); j++)
			res[i][j] = l.getMatrix()[i][j] + r.getMatrix()[i][j];
	}
	return (Matrix<K>(res));
}

template <typename K>
Matrix<K>	operator-(const Matrix<K> &l, const Matrix<K> &r)
{
	if (l.getRowSize() != r.getRowSize() || l.getColumnSize() != r.getColumnSize())
	{
		std::string	msg = "error: cannot add matrices of different sizes";
		throw (msg);
	}

	std::vector<std::vector<K>>	res(l.getRowSize(), std::vector<K>(l.getColumnSize()));
	for (size_t i = 0; i < l.getRowSize(); i++)
	{
		for (size_t j = 0; j < l.getColumnSize(); j++)
			res[i][j] = l.getMatrix()[i][j] - r.getMatrix()[i][j];
	}
	return (Matrix<K>(res));
}

template <typename K, typename T>
Matrix<K>	operator*(const T &l, const Matrix<K> &r)
{
	std::vector<std::vector<K>>	res(r.getRowSize(), std::vector<K>(r.getColumnSize()));

	for (size_t i = 0; i < r.getRowSize(); i++)
	{
		for (size_t j = 0; j < r.getColumnSize(); j++)
			res[i][j] = l * r.getMatrix()[i][j];
	}
	return (Matrix<K>(res));
}

template <typename K, typename T>
Matrix<K>	operator*(const Matrix<K> &l, const T &r)
{
	std::vector<std::vector<K>>	res(l.getRowSize(), std::vector<K>(l.getColumnSize()));

	for (size_t i = 0; i < l.getRowSize(); i++)
	{
		for (size_t j = 0; j < l.getColumnSize(); j++)
			res[i][j] = l.getMatrix()[i][j] * r;
	}
	return (Matrix<K>(res));
}

template <typename K>
Vector<K>	operator*(const Matrix<K> &l, const Vector<K> &r)
{
	if (l.getColumnSize() != r.getSize())
	{
		std::string	msg = "error: cannot multiply m by n matrices with non-n-demension vector";
		throw (msg);
	}

	std::vector<K>	res(l.getRowSize());

	for (size_t m = 0; m < l.getRowSize(); m++)
	{
		for (size_t n = 0; n < l.getColumnSize(); n++)
			res[m] += l.getMatrix()[m][n] * r.getVector()[n];
	}
	return (Vector<K>(res));
}

template <typename K>
Matrix<K>	operator*(const Matrix<K> &l, const Matrix<K> &r)
{
	if (l.getColumnSize() != r.getRowSize())
	{
		std::string	msg = "error: cannot multiply m by n matrix with non-n-by-p matrix";
		throw (msg);
	}

	std::vector<std::vector<K>>	res(l.getRowSize(), std::vector<K>(r.getColumnSize()));
	for (size_t m = 0; m < l.getRowSize(); m++)
	{
		for (size_t p = 0; p < r.getColumnSize(); p++)
		{
			for (size_t n = 0; n < l.getColumnSize(); n++)
				res[m][p] += l.getMatrix()[m][n] * r.getMatrix()[n][p];
		}
	}
	return (Matrix<K>(res));
}

template <typename K, typename T>
Matrix<K>	operator/(const Matrix<K> &l, const T &r)
{
	std::vector<std::vector<K>>	res(l.getRowSize(), std::vector<K>(l.getColumnSize()));

	for (size_t i = 0; i < l.getRowSize(); i++)
	{
		for (size_t j = 0; j < l.getColumnSize(); j++)
			res[i][j] = l.getMatrix()[i][j] / r;
	}
	return (Matrix<K>(res));
}

template <typename K>
std::ostream	&operator<<(std::ostream &ostream, const Matrix<K> &matrix)
{
	ostream << "[ ";
	for (size_t r = 0; r < matrix.getRowSize(); r++)
	{
		for (size_t c = 0; c < matrix.getColumnSize(); c++)
		{
			ostream << matrix.getMatrix()[r][c];
			if (c < matrix.getColumnSize() - 1)
				ostream << " ";
		}
		if (r < matrix.getRowSize() - 1)
			ostream << std::endl << "  ";
	}
	ostream << " ]";
	return (ostream);
}
