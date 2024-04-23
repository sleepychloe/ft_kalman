/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Linear.hpp                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/06 01:14:00 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/14 18:15:59 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef LINEAR_HPP
# define LINEAR_HPP

#include "./Vector.hpp"
#include "./Matrix.hpp"

template <typename K>
Vector<K>		linearCombination(const std::vector<Vector<K>> &u, const Vector<K> &cofs);

template <typename K>
Vector<K>		lerp(const Vector<K> &u, const Vector<K> &v, const K t);

template <typename K>
Matrix<K>		lerp(const Matrix<K> &u, const Matrix<K> &v, const K t);

#include "../srcs/Linear.cpp"

#endif
