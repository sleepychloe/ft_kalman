/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Parse.cpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 17:16:52 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/28 02:15:59 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"

Parse::Parse(): _speed(0)
{
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

std::vector<double>	Parse::getVelocity(void) const
{
	return (this->_velocity);
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

void	Parse:: computeVelocity(void)
{
	double	roll = this->_pos[0];
	double	pitch = this->_pos[1];
	double	yaw = this->_pos[2];

	Matrix<double>	rotation_matrix({
	{std::cos(yaw) * std::cos(pitch), std::cos(yaw) * std::sin(pitch) * std::sin(roll) - std::sin(yaw) * std::cos(roll), std::cos(yaw) * std::sin(pitch) * std::cos(roll) + std::sin(yaw) * std::sin(roll)},
	{std::sin(yaw) * std::cos(pitch), std::sin(yaw) * std::sin(pitch) * std::sin(roll) + std::cos(yaw) * std::cos(roll), std::sin(yaw) * std::sin(pitch) * std::cos(roll) - std::cos(yaw) * std::sin(roll)},
	{-std::sin(pitch), std::cos(pitch) * std::sin(roll), std::cos(pitch) * std::cos(roll)}});

	this->_velocity.clear();
	this->_velocity.push_back(this->_acc[0] * 0.01 * rotation_matrix.getMatrix()[0][2]);
	this->_velocity.push_back(this->_acc[1] * 0.01 * rotation_matrix.getMatrix()[1][2]);
	this->_velocity.push_back(this->_acc[2] * 0.01 * rotation_matrix.getMatrix()[2][2]);
}

void	Parse::print(void) const
{
	std::cout << CYAN << "TRUE POSITION" << BLACK << std::endl;
	for (size_t i = 0; i < this->_pos.size(); i++)
	{
		std::cout << std::setprecision(std::numeric_limits<long double>::digits10) << this->_pos[i];
		if (i < 2)
			std::cout << " ";
	}
	std::cout << std::endl;

	std::cout << CYAN << "SPEED" << BLACK << std::endl;
	std::cout << std::setprecision(std::numeric_limits<long double>::digits10) << this->_speed << std::endl;

	std::cout << CYAN << "ACCELERATION" << BLACK << std::endl;
	for (size_t i = 0; i < this->_acc.size(); i++)
	{
		std::cout << std::setprecision(std::numeric_limits<long double>::digits10) << this->_acc[i];
		if (i < 2)
			std::cout << " ";
	}
	std::cout << std::endl;

	std::cout << CYAN << "DIRECTION" << BLACK << std::endl;
	for (size_t i = 0; i < this->_dir.size(); i++)
	{
		std::cout << std::setprecision(std::numeric_limits<long double>::digits10) << this->_dir[i];
		if (i < 2)
			std::cout << " ";
	}
	std::cout << std::endl;
}
