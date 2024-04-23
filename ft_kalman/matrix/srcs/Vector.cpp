/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Vector.cpp                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/05 18:55:36 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/16 21:01:52 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Vector.hpp"

template <typename K>
Vector<K>::Vector(const std::vector<K> &vector): _size(vector.size()), _vector(vector)
{
}

template <typename K>
Vector<K>::Vector(const Vector<K> &vector)
{
	*this = vector;
}

template <typename K>
Vector<K>& Vector<K>::operator=(const Vector<K> &vector)
{
	if (this == &vector)
		return (*this);
	this->_size = vector._size;
	this->_vector = vector._vector;
	return (*this);
}

template <typename K>
Vector<K>::~Vector()
{
}

template <typename K>
size_t	Vector<K>::getSize(void) const
{
	return (this->_size);
}

template <typename K>
std::vector<K>	Vector<K>::getVector(void) const
{
	return (this->_vector);
}

template <typename K>
void	Vector<K>::printSize(void) const
{
	std::cout << this->_size << " demension vector" << std::endl;
}

template <typename K>
void	Vector<K>::add(const Vector<K> &vector)
{
	if (this->_size != vector.getSize())
	{
		std::string	msg = "error: cannot add vectors of different sizes";
		throw (msg);
	}
	for (size_t i = 0; i < getSize(); i++)
		this->_vector[i] += vector.getVector()[i];
}

template <typename K>
void	Vector<K>::sub(const Vector<K> &vector)
{
	if (this->_size != vector.getSize())
	{
		std::string	msg = "error: cannot subtract vectors of different sizes";
		throw (msg);
	}
	for (size_t i = 0; i < getSize(); i++)
		this->_vector[i] -= vector.getVector()[i];
}

template <typename K>
void	Vector<K>::scale(const K scalar)
{
	for (size_t i = 0; i < this->_size; i++)
		this->_vector[i] *= scalar;
}

template <typename K>
K	Vector<K>::dot(const Vector<K> &vector) const
{
	if (this->_size != vector.getSize())
	{
		std::string	msg = "error: cannot define dot product of vectors of different sizes";
		throw (msg);
	}

	K	res = K(0);

	if constexpr (std::is_arithmetic<K>::value)
	{
		/* real : Σaₖbₖ */
		for (size_t i = 0; i < this->_size; i++)
			res += this->_vector[i] * vector.getVector()[i];
	}
	else
	{
		/* complex: Σaₖb̅ₖ */
		for (size_t i = 0; i < this->_size; i++)
			res += this->_vector[i] * vector.getVector()[i].conj();
	}
	return (res);
}

template <typename K>
K	Vector<K>::norm_1(void) const
{
	K	res = K(0);

	if constexpr (std::is_arithmetic<K>::value)
	{
		/* real : Σ|aₖ| */
		for (size_t i = 0; i < this->_size; i++)
			res += pow(pow(this->_vector[i], 2), 0.5);
	}
	else
	{
		/* complex : Σ|aᵣ|+|aᵢ| */
		for (size_t i = 0; i < this->_size; i++)
		{
			res += pow(pow(this->_vector[i].real(), 2), 0.5);
			res += pow(pow(this->_vector[i].imag(), 2), 0.5);
		}
	}
	return (res);
}

template <typename K>
K	Vector<K>::norm(void) const
{
	K	res = K(0);

	if constexpr (std::is_arithmetic<K>::value)
	{
		/* real : sqrt(Σaₖ²) */
		for (size_t i = 0; i < this->_size; i++)
			res += pow(this->_vector[i], 2);
		return (pow(res, 0.5));
	}
	else
	{
		/* complex : sqrt(Σaᵣ²+aᵢ²) */
		for (size_t i = 0; i < this->_size; i++)
			res += pow(this->_vector[i].real(), 2) + pow(this->_vector[i].imag(), 2);
		return (K(pow(res.real(), 0.5), 0));
	}
}

template <typename K>
K	Vector<K>::norm_inf(void) const
{
	K	res;

	if constexpr (std::is_arithmetic<K>::value)
	{
		/* real : max(|aₖ|) */
		res = pow(pow(this->_vector[0], 2), 0.5);

		for (size_t i = 1; i < this->_size; i++)
			res = pow(std::max(pow(res, 2), pow(this->_vector[i], 2)), 0.5);
		return (res);
	}
	else
	{
		/* complex : max(|aₖ|, |aᵢ|) */
		res = K(pow(pow(this->_vector[0].real(), 2), 0.5), 0);

		for (size_t i = 0; i < this->_size; i++)
		{
			if (res.real() < pow(pow(this->_vector[i].real(), 2), 0.5))
				res = K(pow(pow(this->_vector[i].real(), 2), 0.5), 0);
			if (res.real() < pow(pow(this->_vector[i].imag(), 2), 0.5))
				res = K(pow(pow(this->_vector[i].imag(), 2), 0.5), 0);
		}
		return (res);
	}
}

template <typename K>
Vector<K>	Vector<K>::conjugate(void) const
{
	std::vector<K>	res = this->_vector;

	if constexpr (!std::is_arithmetic<K>::value)
	{
		for (size_t i = 0; i < this->_size; i++)
			res[i] = res[i].conj();
	}
	return (res);
}

template <typename K>
K	angle_cos(const Vector<K> &u, const Vector<K> &v)
{
	if (u.getSize() != v.getSize())
	{
		std::string	msg = "error: cannot calculate cosine between vectors of different sizes";
		throw (msg);
	}

	K	res;

	res = u.dot(v) / (u.norm() * v.norm());
	return (res);
}

template <typename K>
Vector<K>	corss_product(const Vector<K> &u, const Vector<K> &v)
{
	if (!(u.getSize() == v.getSize() && u.getSize() == 3))
	{
		std::string	msg = "error: cannot define cross product of non-3-demension vectors";
		throw (msg);
	}

	std::vector<K>	res(u.getSize());

	res[0] = u.getVector()[1] * v.getVector()[2] - u.getVector()[2] * v.getVector()[1];
	res[1] = u.getVector()[2] * v.getVector()[0] - u.getVector()[0] * v.getVector()[2];
	res[2] = u.getVector()[0] * v.getVector()[1] - u.getVector()[1] * v.getVector()[0];
	return (Vector<K>(res));
}

template <typename K>
std::ostream	&operator<<(std::ostream &ostream, const Vector<K> &vector)
{
	ostream << "( ";
	for (size_t i = 0; i < vector.getSize(); i++)
	{
		ostream << vector.getVector()[i];
		if (i < vector.getSize() - 1)
			ostream << ", ";
	}
	ostream << " )";
	return (ostream);
}
