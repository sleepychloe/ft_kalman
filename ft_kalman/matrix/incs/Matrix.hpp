/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Matrix.hpp                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/05 02:18:09 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/16 05:00:37 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef MATRIX_HPP
# define MATRIX_HPP

#include <vector>
#include <iostream>
#include <string>
#include <functional>
#include "./Vector.hpp"
#include "./Complex.hpp"

# define EPSILON			1e-7

template <typename K>
class Matrix
{
public:
	Matrix(const std::vector<std::vector<K>> &matrix);
	Matrix(const Matrix<K> &matrix);
	Matrix &operator=(const Matrix<K> &matrix);
	~Matrix();

	size_t				getRowSize(void) const;
	size_t				getColumnSize(void) const;
	std::vector<std::vector<K>>	getMatrix(void) const;
	bool				isSquare(void) const;
	void				printSize(void) const;

	void				add(const Matrix<K> &vector);
	void				sub(const Matrix<K> &vector);
	void				scale(const K scalar);
	Vector<K>			mul_vec(const Vector<K> &vector) const;
	Matrix<K>			mul_mat(const Matrix<K> &matrix) const;
	K				trace(void) const;
	Matrix<K>			transpose(void) const;
	Matrix<K>			conjugate(void) const;
	Matrix<K>			conjugateTranspose(void) const;
	Matrix<K>			row_echelon(void) const;
	Matrix<K>			minor(size_t m, size_t n) const;
	K				determinant(void) const;
	Matrix<K>			cofactor(void) const;
	Matrix<K>			inverse(void) const;
	size_t				rank(void) const;

private:
	Matrix();

	size_t				_row;
	size_t				_column;
	std::vector<std::vector<K>>	_matrix;

	void				rowOperation_1(std::vector<std::vector<K>> *m, size_t r1, size_t r2) const;
	void				rowOperation_2(std::vector<std::vector<K>> *m, size_t r, K k) const;
	void				rowOperation_3(std::vector<std::vector<K>> *m, size_t r1, K k, size_t r2) const;
};

template <typename K>
Matrix<K>			projection(K fov, K ratio, K near, K far);

template <typename K>
std::ostream	&operator<<(std::ostream &ostream, const Matrix<K> &matrix);

#include "../srcs/Matrix.cpp"

#endif
