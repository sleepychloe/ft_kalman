/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Parse.hpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 17:16:44 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/10 21:33:23 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef PARSE_HPP
# define PARSE_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <cmath>
#include "./Color.hpp"

class Parse
{
public:
	Parse(int duration);
	Parse(const Parse &parse);
	Parse &operator=(const Parse &parse);
	~Parse();

	std::vector<double>		getPos(void) const;
	double				getSpeed(void) const;
	std::vector<double>		getAcc(void) const;
	std::vector<double>		getDir(void) const;
	std::string			getEndtime(void) const;

	void				parse(std::string &buf);

private:
	void				parseVec(std::string &buf, std::vector<double> &data);
	void				parseScala(std::string &buf, double &data);

	std::vector<double>		_pos;
	double				_speed;
	std::vector<double>		_acc;
	std::vector<double>		_dir;
	std::string			_endtime;
};

#endif
