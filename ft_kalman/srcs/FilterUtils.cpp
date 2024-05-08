/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   FilterUtils.cpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/02 07:58:26 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/08 16:42:37 by yhwang           ###   ########.fr       */
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

Matrix<double>	integrate(Matrix<double> m, double start, double end)
{
	size_t	n = 10000;
	double	dx = (end - start) / n;

	std::vector<std::vector<double>>	res(m.getRowSize(), std::vector<double>(m.getColumnSize()));
	for (size_t i = 0; i < n; i++)
	{
		for (size_t r = 0; r < m.getRowSize(); r++)
		{
			for (size_t c = 0; c < m.getColumnSize(); c++)
				res[r][c] += m.getMatrix()[r][c] * dx;
		}
	}
	return (Matrix<double>(res));
}

void	initFilter(Parse &p, std::vector<double> v, KalmanFilter<double> &kalman)
{
	/* init state: n(pos, v) */
	Vector<double>	init_state({p.getPos()[0], p.getPos()[1], p.getPos()[2], v[0], v[1], v[2]});

	/* init covariance: n by n */
	Matrix<double>	init_covariance({{pow(GPS_NOISE, 2), 0, 0, 0, 0, 0},
					{0, pow(GPS_NOISE, 2), 0, 0, 0, 0},
					{0, 0, pow(GPS_NOISE, 2), 0, 0, 0},
					{0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT), 0, 0},
					{0, 0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT), 0},
					{0, 0, 0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT)}});

	/* transition: n by n */
	Matrix<double>	transition_matrix({{1, 0, 0, DT, 0, 0},
						{0, 1, 0, 0, DT, 0},
						{0, 0, 1, 0, 0, DT},
						{0, 0, 0, 1, 0, 0},
						{0, 0, 0, 0, 1, 0},
						{0, 0, 0, 0, 0, 1}});

	/* control model: n by n */
	Matrix<double>	control_transition_model({{DT * DT / 2, 0, 0},
						{0, DT * DT / 2, 0},
						{0, 0, DT * DT / 2},
						{DT, 0, 0},
						{0, DT, 0},
						{0, 0, DT}});

	/* observation: m by n */
	Matrix<double>	observation_matrix({{1, 0, 0, 0, 0, 0},
						{0, 1, 0, 0, 0, 0},
						{0, 0, 1, 0, 0, 0}});

	/* process_noise: n by n */
	// Matrix<double>	q_continuous({{DT * DT / 2, 0, 0, DT * DT / 2, 0, 0},
	// 				{0, DT * DT / 2, 0, 0, DT * DT / 2, 0},
	// 				{0, 0, DT * DT / 2, 0, 0, DT * DT / 2},
	// 				{0, 0, 0, DT, 0, 0},
	// 				{0, 0, 0, 0, DT, 0},
	// 				{0, 0, 0, 0, 0, DT}});
	// double		variance_p = pow(GPS_NOISE, 2);
	// double		variance_v = pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT;
	// double		variance_a = pow(ACCELEROMETER_NOISE, 2);
	// Matrix<double>	noise_p({{variance_p, 0, 0, 0, 0, 0},
	// 				{0, variance_p, 0, 0, 0, 0},
	// 				{0, 0, variance_p, 0, 0, 0},
	// 				{0, 0, 0, 0, 0, 0},
	// 				{0, 0, 0, 0, 0, 0},
	// 				{0, 0, 0, 0, 0, 0,}});
	// Matrix<double>	noise_v({{0, 0, 0, variance_v * DT, 0, 0},
	// 				{0, 0, 0, 0, variance_v * DT, 0},
	// 				{0, 0, 0, 0, 0, variance_v * DT},
	// 				{0, 0, 0, variance_v, 0, 0},
	// 				{0, 0, 0, 0, variance_v, 0},
	// 				{0, 0, 0, 0, 0, variance_v}});
	// Matrix<double>	noise_a({{variance_a * DT * DT / 2, 0, 0, variance_a * DT * DT / 2, 0, 0},
	// 				{0, variance_a * DT * DT / 2, 0, 0, variance_a * DT * DT / 2, 0},
	// 				{0, 0, variance_a * DT * DT / 2, 0, 0, variance_a * DT * DT / 2},
	// 				{0, 0, 0, variance_a * DT, 0, 0},
	// 				{0, 0, 0, 0, variance_a * DT, 0},
	// 				{0, 0, 0, 0, 0, variance_a * DT}});
	// Matrix<double>	noise_density = noise_p + noise_v + noise_a;
	// q_continuous = q_continuous * noise_density;
	// Matrix<double>	process_noise_covariance = transition_matrix * q_continuous * transition_matrix.transpose();
	// process_noise_covariance = integrate(process_noise_covariance, 0, DT);
	Matrix<double>	q_continuous_p({{DT * DT / 2, 0, 0, 0, 0, 0},
					{0, DT * DT / 2, 0, 0, 0, 0},
					{0, 0, DT * DT / 2, 0, 0, 0},
					{0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0}});
	Matrix<double>	q_continuous_v({{0, 0, 0, DT * DT / 2, 0, 0},
					{0, 0, 0, 0, DT * DT / 2, 0},
					{0, 0, 0, 0, 0, DT * DT / 2},
					{0, 0, 0, DT, 0, 0},
					{0, 0, 0, 0, DT, 0},
					{0, 0, 0, 0, 0, DT}});
	Matrix<double>	q_continuous_a({{DT * DT / 2, 0, 0, DT * DT / 2, 0, 0},
					{0, DT * DT / 2, 0, 0, DT * DT / 2, 0},
					{0, 0, DT * DT / 2, 0, 0, DT * DT / 2},
					{0, 0, 0, DT, 0, 0},
					{0, 0, 0, 0, DT, 0},
					{0, 0, 0, 0, 0, DT}});
	double		variance_p = pow(GPS_NOISE, 2);
	double		variance_v = pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT;
	double		variance_a = pow(ACCELEROMETER_NOISE, 2);
	Matrix<double>	noise_p({{variance_p, 0, 0, 0, 0, 0},
					{0, variance_p, 0, 0, 0, 0},
					{0, 0, variance_p, 0, 0, 0},
					{0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0,}});
	Matrix<double>	noise_v({{0, 0, 0, variance_v * DT, 0, 0},
					{0, 0, 0, 0, variance_v * DT, 0},
					{0, 0, 0, 0, 0, variance_v * DT},
					{0, 0, 0, variance_v, 0, 0},
					{0, 0, 0, 0, variance_v, 0},
					{0, 0, 0, 0, 0, variance_v}});
	Matrix<double>	noise_a({{variance_a * DT * DT / 2, 0, 0, variance_a * DT * DT / 2, 0, 0},
					{0, variance_a * DT * DT / 2, 0, 0, variance_a * DT * DT / 2, 0},
					{0, 0, variance_a * DT * DT / 2, 0, 0, variance_a * DT * DT / 2},
					{0, 0, 0, variance_a * DT, 0, 0},
					{0, 0, 0, 0, variance_a * DT, 0},
					{0, 0, 0, 0, 0, variance_a * DT}});
	q_continuous_p = q_continuous_p * noise_p;
	q_continuous_v = q_continuous_v * noise_v;
	q_continuous_a = q_continuous_a * noise_a;
	Matrix<double>	q_continuous = q_continuous_p + q_continuous_v + q_continuous_a;
	Matrix<double>	process_noise_covariance = transition_matrix * q_continuous * transition_matrix.transpose();
	process_noise_covariance = integrate(process_noise_covariance, 0, DT);

	/* measurement noise: m by m */
	Matrix<double>	measurement_noise_covariance({{variance_p, 0, 0},
						{0, variance_p, 0},
						{0, 0, variance_p}});

	kalman = KalmanFilter<double>(init_state, init_covariance,
					transition_matrix, observation_matrix, control_transition_model,
					process_noise_covariance, measurement_noise_covariance);
}
