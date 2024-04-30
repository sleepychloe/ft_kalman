/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Parse.cpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 17:16:52 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/30 06:29:17 by yhwang           ###   ########.fr       */
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

	Matrix<double>	rotation_x({{1, 0, 0},
					{0, std::cos(roll), -1 * std::sin(roll)},
					{0, std::sin(roll), std::cos(roll)}});
	Matrix<double>	rotation_y({{std::cos(pitch), 0, std::sin(pitch)},
					{0, 1, 0},
					{-1 * std::sin(pitch), 0, std::cos(pitch)}});
	Matrix<double>	rotation_z({{std::cos(yaw), -1 * std::sin(yaw), 0},
					{std::sin(yaw), std::cos(yaw), 0},
					{0, 0, 1}});
	Matrix<double>	rotation = rotation_z * rotation_y * rotation_x;
	
	double	acc_x = rotation.getMatrix()[0][0] * this->getAcc()[0] + rotation.getMatrix()[0][1] * this->getAcc()[1] + rotation.getMatrix()[0][2] * this->getAcc()[2];
	double	acc_y = rotation.getMatrix()[1][0] * this->getAcc()[0] + rotation.getMatrix()[1][1] * this->getAcc()[1] + rotation.getMatrix()[1][2] * this->getAcc()[2];
	double	acc_z = rotation.getMatrix()[2][0] * this->getAcc()[0] + rotation.getMatrix()[2][1] * this->getAcc()[1] + rotation.getMatrix()[2][2] * this->getAcc()[2];

	if (this->_velocity.size() == 0)
	{
		this->_velocity.push_back(this->_speed + acc_x * 0.01);
		this->_velocity.push_back(acc_y * 0.01);
		this->_velocity.push_back(acc_z * 0.01);
	}
	else
	{
		this->_velocity[0] += acc_x * 0.01;
		this->_velocity[1] += acc_y * 0.01;
		this->_velocity[2] += acc_z * 0.01;
	}
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
