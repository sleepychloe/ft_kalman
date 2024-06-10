/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Parse.cpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 17:16:52 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/10 21:39:21 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"

Parse::Parse(int duration): _speed(0)
{
	std::string	h = "00";
	std::string	m;
	std::string	s = "59.970";
	std::string	endtime;

	if (duration <= 60)
	{
		if (duration < 10)
			m = "0";
		m += std::to_string(duration - 1);
	}
	else
	{
		h = "01";
		if (duration - 60 < 10)
			m = "0";
		m += std::to_string(duration - 61);
	}

	endtime = "[" + h + ":" + m + ":" + s + "]";
	this->_endtime = endtime;
}

Parse::Parse(const Parse &parse)
{
	*this = parse;
}

Parse &Parse::operator=(const Parse &parse)
{
	if (this == &parse)
		return (*this);
	this->_pos = parse._pos;
	this->_speed = parse._speed;
	this->_acc = parse._acc;
	this->_dir = parse._dir;
	return (*this);
}

Parse::~Parse()
{
}

std::vector<double>	Parse::getPos(void) const
{
	return (this->_pos);
}

double	Parse::getSpeed(void) const
{
	return (this->_speed);
}

std::vector<double>	Parse::getAcc(void) const
{
	return (this->_acc);
}

std::vector<double>	Parse::getDir(void) const
{
	return (this->_dir);
}

std::string	Parse::getEndtime(void) const
{
	return (this->_endtime);
}

void	Parse::parseVec(std::string &buf, std::vector<double> &data)
{
	buf = buf.substr(buf.find('\n') + 1, std::string::npos);
	std::stringstream	ss(buf);
	std::string		tmp[3];

	data.clear();
	for (int i = 0; i < 3; i++)
	{
		getline(ss, tmp[i], '\n');
		data.push_back(stold(tmp[i]));
	}
}

void	Parse::parseScala(std::string &buf, double &data)
{
	buf = buf.substr(buf.find('\n') + 1, std::string::npos);
	std::stringstream	ss(buf);
	std::string		tmp;

	getline(ss, tmp, '\n');
	data = stold(tmp) * 10 / 36;
}

void	Parse::parse(std::string &buf)
{
	if (buf.find("POSITION") != std::string::npos)
		parseVec(buf, this->_pos);
	else if (buf.find("SPEED") != std::string::npos)
		parseScala(buf, this->_speed);
	else if (buf.find("ACCELERATION") != std::string::npos)
		parseVec(buf, this->_acc);
	else if (buf.find("DIRECTION") != std::string::npos)
		parseVec(buf, this->_dir);
}
