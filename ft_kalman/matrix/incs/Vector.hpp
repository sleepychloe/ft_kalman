/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Vector.hpp                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/05 02:18:30 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/15 22:00:03 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef VECTOR_HPP
# define VECTOR_HPP

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "./Complex.hpp"

template <typename K>
class Vector
{
public:
	Vector(const std::vector<K> &vector);
	Vector(const Vector<K> &vector);
	Vector& operator=(const Vector<K> &vector);
	~Vector();

	size_t				getSize(void) const;
	std::vector<K>			getVector(void) const;
	void				printSize(void) const;

	void				add(const Vector<K> &vector);
	void				sub(const Vector<K> &vector);
	void				scale(const K scalar);
	K				dot(const Vector<K> &vector) const;
	K				norm_1(void) const;
	K				norm(void) const;
	K				norm_inf(void) const;
	Vector<K>			conjugate(void) const;
	
private:
	Vector();

	size_t				_size;
	std::vector<K>			_vector;
};

template <typename K>
K		angle_cos(const Vector<K> &u, const Vector<K> &v);

template <typename K>
Vector<K>	corss_product(const Vector<K> &u, const Vector<K> &v);

template <typename K>
std::ostream	&operator<<(std::ostream &ostream, const Vector<K> &vector);

#include "../srcs/Vector.cpp"

#endif
