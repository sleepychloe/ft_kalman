/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Parse.hpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 17:16:44 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/02 08:18:25 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef PARSE_HPP
# define PARSE_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <cmath>

#include "./Color.hpp"

# define	POS		1
# define	SPEED		2
# define 	ACC		3
# define 	DIR		4

class Parse
{
public:
	Parse();
	Parse(const Parse &parse);
	Parse &operator=(const Parse &parse);
	~Parse();

	std::vector<double>		getPos(void) const;
	double				getSpeed(void) const;
	std::vector<double>		getAcc(void) const;
	std::vector<double>		getDir(void) const;

	void				parse(std::string &buf);
	void				print(void) const;

private:
	void				parseVec(std::string &buf, std::vector<double> &data);
	void				parseScala(std::string &buf, double &data);

	std::vector<double>		_pos;
	double				_speed;
	std::vector<double>		_acc;
	std::vector<double>		_dir;
};

#endif
