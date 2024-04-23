/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Complex.hpp                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/13 15:54:24 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/16 20:52:48 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef COMPLEX_HPP
# define COMPLEX_HPP

#include <iostream>
#include <type_traits>

template <typename K>
class Complex
{
public:
	Complex();
	Complex(const K &real, const K &imag);
	Complex(const K &num);
	Complex(const Complex<K> &complex);
	Complex &operator=(const Complex<K> &complex);
	~Complex();

	Complex<K>	&operator+=(const K &num);
	Complex<K>	&operator-=(const K &num);
	Complex<K>	&operator*=(const K &num);
	Complex<K>	&operator/=(const K &num);
	Complex<K>	&operator+=(const Complex<K> &complex);
	Complex<K>	&operator-=(const Complex<K> &complex);
	Complex<K>	&operator*=(const Complex<K> &complex);
	Complex<K>	&operator/=(const Complex<K> &complex);	

	K				real(void) const;
	K				imag(void) const;

	Complex<K>			conj(void) const;

private:
	K				_real;
	K				_imag;
};

template <typename K>
std::ostream	&operator<<(std::ostream &ostream, const Complex<K> &complex);

template <typename K>
Complex<K>	operator+(const Complex<K> &l, const Complex<K> &r);

template <typename K, typename T>
Complex<K>	operator+(const Complex<K> &l, const T &r);

template <typename K, typename T>
Complex<K>	operator+(const T &l, const Complex<K> &r);

template <typename K>
Complex<K>	operator-(const Complex<K> &l, const Complex<K> &r);

template <typename K, typename T>
Complex<K>	operator-(const Complex<K> &l, const T &r);

template <typename K, typename T>
Complex<K>	operator-(const T &l, const Complex<K> &r);

template <typename K>
Complex<K>	operator*(const Complex<K> &l, const Complex<K> &r);

template <typename K, typename T>
Complex<K>	operator*(const Complex<K> &l, const T &r);

template <typename K, typename T>
Complex<K>	operator*(const T &l, const Complex<K> &r);

template <typename K>
Complex<K>	operator/(const Complex<K> &l, const Complex<K> &r);

template <typename K, typename T>
Complex<K>	operator/(const Complex<K> &l, const T &r);

template <typename K, typename T>
Complex<K>	operator/(const T &l, const Complex<K> &r);

template <typename K>
bool		operator==(const Complex<K> &l, const int &r);

template <typename K>
bool		operator==(const int &l, const Complex<K> &r);

template <typename K>
bool		operator==(const Complex<K> &l, Complex<K> &r);

template <typename K>
bool		operator!=(const Complex<K> &l, const int &r);

template <typename K>
bool		operator!=(const int &l, const Complex<K> &r);

template <typename K>
bool		operator!=(const Complex<K> &l, Complex<K> &r);

template <typename K>
bool		operator<(const Complex<K> &l, const int &r);

template <typename K>
bool		operator<(const int &l, const Complex<K> &r);

template <typename K>
bool		operator<(const Complex<K> &l, Complex<K> &r);

template <typename K>
bool		operator>(const Complex<K> &l, const int &r);

template <typename K>
bool		operator>(const int &l, const Complex<K> &r);

template <typename K>
bool		operator>(const Complex<K> &l, Complex<K> &r);

template <typename K>
bool		operator<=(const Complex<K> &l, const int &r);

template <typename K>
bool		operator<=(const int &l, const Complex<K> &r);

template <typename K>
bool		operator<=(const Complex<K> &l, Complex<K> &r);

template <typename K>
bool		operator>=(const Complex<K> &l, const int &r);

template <typename K>
bool		operator>=(const int &l, const Complex<K> &r);

template <typename K>
bool		operator>=(const Complex<K> &l, Complex<K> &r);

#include "../srcs/Complex.cpp"

#endif
