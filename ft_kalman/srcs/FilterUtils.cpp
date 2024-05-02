/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   FilterUtils.cpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/02 07:58:26 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/02 09:40:17 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Utils.hpp"

void	computeVelocity(std::vector<double> d,
			std::vector<double> a, std::vector<double> &v)
{
	double	roll = d[0];
	double	pitch = d[1];
	double	yaw = d[2];

	Matrix<double>	r_x({{1, 0, 0},
					{0, std::cos(roll), -1 * std::sin(roll)},
					{0, std::sin(roll), std::cos(roll)}});
	Matrix<double>	r_y({{std::cos(pitch), 0, std::sin(pitch)},
					{0, 1, 0},
					{-1 * std::sin(pitch), 0, std::cos(pitch)}});
	Matrix<double>	r_z({{std::cos(yaw), -1 * std::sin(yaw), 0},
					{std::sin(yaw), std::cos(yaw), 0},
					{0, 0, 1}});
	Matrix<double>	r = r_z * r_y * r_x;
	
	double	a_x = r.getMatrix()[0][0] * a[0] + r.getMatrix()[0][1] * a[1] + r.getMatrix()[0][2] * a[2];
	double	a_y = r.getMatrix()[1][0] * a[0] + r.getMatrix()[1][1] * a[1] + r.getMatrix()[1][2] * a[2];
	double	a_z = r.getMatrix()[2][0] * a[0] + r.getMatrix()[2][1] * a[1] + r.getMatrix()[2][2] * a[2];

	v[0] += a_x * DT;
	v[1] += a_y * DT;
	v[2] += a_z * DT;
}

void	initFilter(Parse &p, std::vector<double> v, KalmanFilter<double> &kalman)
{
	// init state: 1 * n: pos, v, acc
	Vector<double>	init_state({p.getPos()[0], p.getPos()[1], p.getPos()[2], v[0], v[1], v[2], p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});

	// init covariance: n * n
	Matrix<double>	init_covariance({{1, 0, 0, 0, 0, 0, 0, 0, 0},
					{0, 1, 0, 0, 0, 0, 0, 0, 0},
					{0, 0, 1, 0, 0, 0, 0, 0, 0},
					{0, 0, 0, 1, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 1, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 1, 0, 0, 0},
					{0, 0, 0, 0, 0, 0, 1, 0, 0},
					{0, 0, 0, 0, 0, 0, 0, 1, 0},
					{0, 0, 0, 0, 0, 0, 0, 0, 1}});

	// transition: n * n
	Matrix<double>	transition_matrix({{1, 0, 0, DT, 0, 0, DT * DT / 2, 0, 0},
						{0, 1, 0, 0, DT, 0, 0, DT * DT / 2, 0},
						{0, 0, 1, 0, 0, DT, 0, 0, DT * DT / 2},
						{0, 0, 0, 1, 0, 0, DT, 0, 0},
						{0, 0, 0, 0, 1, 0, 0, DT, 0},
						{0, 0, 0, 0, 0, 1, 0, 0, DT},
						{0, 0, 0, 0, 0, 0, 1, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 1, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 1}});

	// observation: m * n
	Matrix<double>	observation_matrix({{1, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 1, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 1, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 1, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 1, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 1}});

	//[00:12:56.530]
	Matrix<double>	process_noise_covariance({{pow(GPS_NOISE, 2) * DT * DT * DT / 6, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, pow(GPS_NOISE, 2) * DT * DT * DT / 6, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, pow(GPS_NOISE, 2) * DT * DT * DT / 6, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT) * DT * DT / 2, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT) * DT * DT / 2, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT) * DT * DT / 2, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE, 2) * DT, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE, 2) * DT, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE, 2) * DT}});

	// measurement noise: m * m
	Matrix<double>	measurement_noise_covariance({{pow(GPS_NOISE, 2), 0, 0, 0, 0, 0},
						{0, pow(GPS_NOISE, 2), 0, 0, 0, 0},
						{0, 0, pow(GPS_NOISE, 2), 0, 0, 0},
						{0, 0, 0, pow(ACCELEROMETER_NOISE, 2), 0, 0},
						{0, 0, 0, 0, pow(ACCELEROMETER_NOISE, 2), 0},
						{0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE, 2)}});

	kalman = KalmanFilter<double>(init_state, init_covariance,
					transition_matrix, observation_matrix,
					process_noise_covariance, measurement_noise_covariance);
}
