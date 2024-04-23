/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Complex.cpp                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/13 16:04:41 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/16 21:13:33 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Complex.hpp"

template <typename K>
Complex<K>::Complex(): _real(0), _imag(0)
{
}

template <typename K>
Complex<K>::Complex(const K &num): _real(num), _imag(num)
{
}

template <typename K>
Complex<K>::Complex(const K &real, const K &imag): _real(0), _imag(0)
{
	this->_real = real;
	this->_imag = imag;
}

template <typename K>
Complex<K>::Complex(const Complex<K> &complex)
{
	*this = complex;
}

template <typename K>
Complex<K>&	Complex<K>::operator=(const Complex<K> &complex)
{
	if (this == &complex)
		return (*this);
	this->_real = complex._real;
	this->_imag = complex._imag;
	return (*this);
}

template <typename K>
Complex<K>::~Complex()
{
}

template <typename K>
Complex<K>	&Complex<K>::operator+=(const K &num)
{
	this->_real += num;
	return (*this);
}

template <typename K>
Complex<K>	&Complex<K>::operator-=(const K &num)
{
	this->_real -= num;
	return (*this);
}

template <typename K>
Complex<K>	&Complex<K>::operator*=(const K &num)
{
	this->_real *= num;
	this->_imag *= num;
	return (*this);
}

template <typename K>
Complex<K>	&Complex<K>::operator/=(const K &num)
{
	this->_real /= num;
	this->_imag /= num;
	return (*this);
}

template <typename K>
Complex<K>	&Complex<K>::operator+=(const Complex<K> &complex)
{
	this->_real += complex.real();
	this->_imag += complex.imag();
	return (*this);
}

template <typename K>
Complex<K>	&Complex<K>::operator-=(const Complex<K> &complex)
{
	this->_real -= complex.real();
	this->_imag -= complex.imag();
	return (*this);
}

template <typename K>
Complex<K>	&Complex<K>::operator*=(const Complex<K> &complex)
{
	K	real = this->_real * complex.real() - this->_imag * complex.imag();
	K	imag = this->_real * complex.imag() + this->_imag * complex.real();

	this->_real = real;
	this->_imag = imag;
	return (*this);
}

template <typename K>
Complex<K>	&Complex<K>::operator/=(const Complex<K> &complex)
{
	K	denominator = complex.real() * complex.real()
				+ complex.imag() * complex.imag();
	K	real = this->_real * complex.real() + this->_imag * complex.imag();
	K	imag = -1 * this->_real * complex.imag() + this->_imag * complex.real();

	this->_real = real / denominator;
	this->_imag = imag / denominator;
	return (*this);
}

template <typename K>
K	Complex<K>::real(void) const
{
	return (this->_real);
}

template <typename K>
K	Complex<K>::imag(void) const
{
	return (this->_imag);
}

template <typename K>
Complex<K> Complex<K>::conj(void) const
{
	K	real;
	K	imag;

	real = this->_real;
	imag = -1 * this->_imag;

	return  (Complex<K>(real, imag));
}

template <typename K>
std::ostream	&operator<<(std::ostream &ostream, const Complex<K> &complex)
{
	if (complex.real() != 0 || (complex.real() == 0 && complex.imag() == 0))
		ostream << complex.real();
	if (complex.imag() != 0)
	{
		if (complex.imag() > 0)
		{
			if (complex.real() != 0)
				ostream << "+";
			ostream << complex.imag() << "i";
		}
		else
			ostream << complex.imag() << "i";
	}
	return (ostream);
}

template <typename K>
Complex<K>	operator+(const Complex<K> &l, const Complex<K> &r)
{
	K	real = l.real() + r.real();
	K	imag = l.imag() + r.imag();
	return (Complex<K>(real, imag));
}

template <typename K, typename T>
Complex<K>	operator+(const Complex<K> &l, const T &r)
{
	if constexpr (std::is_arithmetic<T>::value)
	{
		K	real = l.real() + r;
		K	imag = l.imag();
		return (Complex<K>(real, imag));
	}
}

template <typename K, typename T>
Complex<K>	operator+(const T &l, const Complex<K> &r)
{
	if constexpr (std::is_arithmetic<T>::value)
	{
		K	real = l + r.real();
		K	imag = r.imag();
		return (Complex<K>(real, imag));
	}
}

template <typename K>
Complex<K>	operator-(const Complex<K> &l, const Complex<K> &r)
{
	K	real = l.real() - r.real();
	K	imag = l.imag() - r.imag();
	return (Complex<K>(real, imag));
}

template <typename K, typename T>
Complex<K>	operator-(const Complex<K> &l, const T &r)
{
	if constexpr (std::is_arithmetic<T>::value)
	{
		K	real = l.real() - r;
		K	imag = l.imag();
		return (Complex<K>(real, imag));
	}
}

template <typename K, typename T>
Complex<K>	operator-(const T &l, const Complex<K> &r)
{
	if constexpr (std::is_arithmetic<T>::value)
	{
		K	real = l - r.real();
		K	imag = -1 * r.imag();
		return (Complex<K>(real, imag));
	}
}

template <typename K>
Complex<K>	operator*(const Complex<K> &l, const Complex<K> &r)
{
	K	real = l.real() * r.real() - l.imag() * r.imag();
	K	imag = l.real() * r.imag() + l.imag() * r.real();
	return (Complex<K>(real, imag));
}

template <typename K, typename T>
Complex<K>	operator*(const Complex<K> &l, const T &r)
{
	if constexpr (std::is_arithmetic<T>::value)
	{
		K	real = l.real() * r;
		K	imag = l.imag() * r;
		return (Complex<K>(real, imag));
	}
}

template <typename K, typename T>
Complex<K>	operator*(const T &l, const Complex<K> &r)
{
	if constexpr (std::is_arithmetic<T>::value)
	{
		K	real = l * r.real();
		K	imag = l * r.imag();
		return (Complex<K>(real, imag));
	}
}

template <typename K>
Complex<K>	operator/(const Complex<K> &l, const Complex<K> &r)
{
	K	denominator = r.real() * r.real()
				+ r.imag() * r.imag();
	K	real = l.real() * r.real() + l.imag() * r.imag();
	K	imag = -1 * l.real() * r.imag() + l.imag() * r.real();
	return (Complex<K>(real / denominator, imag / denominator));
}

template <typename K, typename T>
Complex<K>	operator/(const Complex<K> &l, const T &r)
{
	if constexpr (std::is_arithmetic<T>::value)
	{
		K	real = l.real() / r;
		K	imag = l.imag() / r;
		return (Complex<K>(real, imag));
	}
}

template <typename K, typename T>
Complex<K>	operator/(const T &l, const Complex<K> &r)
{
	if constexpr (std::is_arithmetic<T>::value)
	{
		K	denominator = r.real() * r.real()
					+ r.imag() * r.imag();
		K	real = l * r.real();
		K	imag = -1 * l * r.imag();
		return (Complex<K>(real / denominator, imag / denominator));
	}
}

template <typename K>
bool		operator==(const Complex<K> &l, const int &r)
{
	return (l.real() == r && l.imag() == 0);
}

template <typename K>
bool		operator==(const int &l, const Complex<K> &r)
{
	return (l == r.real() && r.imag() == 0);
}

template <typename K>
bool		operator==(const Complex<K> &l, Complex<K> &r)
{
	return (l.real() == r.real() && l.imag() == r.imag());
}

template <typename K>
bool		operator!=(const Complex<K> &l, const int &r)
{
	return (!(l == r));
}

template <typename K>
bool		operator!=(const int &l, const Complex<K> &r)
{
	return (!(l == r));
}

template <typename K>
bool		operator!=(const Complex<K> &l, Complex<K> &r)
{
	return (!(l == r));
}

template <typename K>
bool	operator<(const Complex<K> &l, const int &r)
{
	K	l_square = l.real() * l.real() + l.imag() * l.imag();

	return (l_square < r * r);
}

template <typename K>
bool	operator<(const int &l, const Complex<K> &r)
{
	K	r_square = r.real() * r.real() + r.imag() * r.imag();

	return (l * l < r_square);
}

template <typename K>
bool	operator<(const Complex<K> &l, Complex<K> &r)
{
	K	l_square = l.real() * l.real() + l.imag() * l.imag();
	K	r_square = r.real() * r.real() + r.imag() * r.imag();

	return (l_square < r_square);
}

template <typename K>
bool	operator>(const Complex<K> &l, const int &r)
{
	K	l_square = l.real() * l.real() + l.imag() * l.imag();

	return (l_square > r * r);
}

template <typename K>
bool	operator>(const int &l, const Complex<K> &r)
{
	K	r_square = r.real() * r.real() + r.imag() * r.imag();

	return (l * l > r_square);
}

template <typename K>
bool	operator>(const Complex<K> &l, Complex<K> &r)
{
	K	l_square = l.real() * l.real() + l.imag() * l.imag();
	K	r_square = r.real() * r.real() + r.imag() * r.imag();

	return (l_square > r_square);
}

template <typename K>
bool	operator<=(const Complex<K> &l, const int &r)
{
	K	l_square = l.real() * l.real() + l.imag() * l.imag();

	return (l_square <= r * r);
}

template <typename K>
bool	operator<=(const int &l, const Complex<K> &r)
{
	K	r_square = r.real() * r.real() + r.imag() * r.imag();

	return (l * l <= r_square);
}

template <typename K>
bool	operator<=(const Complex<K> &l, Complex<K> &r)
{
	K	l_square = l.real() * l.real() + l.imag() * l.imag();
	K	r_square = r.real() * r.real() + r.imag() * r.imag();

	return (l_square <= r_square);
}

template <typename K>
bool	operator>=(const Complex<K> &l, const int &r)
{
	K	l_square = l.real() * l.real() + l.imag() * l.imag();

	return (l_square >= r * r);
}

template <typename K>
bool	operator>=(const int &l, const Complex<K> &r)
{
	K	r_square = r.real() * r.real() + r.imag() * r.imag();

	return (l * l >= r_square);
}

template <typename K>
bool	operator>=(const Complex<K> &l, Complex<K> &r)
{
	K	l_square = l.real() * l.real() + l.imag() * l.imag();
	K	r_square = r.real() * r.real() + r.imag() * r.imag();

	return (l_square >= r_square);
}

