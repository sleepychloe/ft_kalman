/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   FilterUtils.hpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/02 07:58:18 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/02 08:26:35 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef FILTER_UTILS_HPP
# define FILTER_UTILS_HPP

#include "../matrix/incs/Vector.hpp"
#include "../matrix/incs/Matrix.hpp"
#include "../incs/Parse.hpp"
#include "../incs/KalmanFilter.hpp"

# define	DT			0.01
# define 	ACCELEROMETER_NOISE	0.001
# define	GYROSCOPE_NOISE		0.01
# define	GPS_NOISE		0.1

void			computeVelocity(std::vector<double> d,
				std::vector<double> a, std::vector<double> &v);
void			initFilter(Parse &p, std::vector<double> v, KalmanFilter<double> &kalman);

#endif
