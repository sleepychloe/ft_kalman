/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   KalmanFilter.cpp                                   :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/25 01:15:54 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/25 22:09:56 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/KalmanFilter.hpp"

template <typename K>
KalmanFilter<K>::KalmanFilter(Vector<K> initial_satate, Matrix<K> initial_covariance,
		Matrix<K> transition_matrix, Matrix<K> observation_matrix,
		Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance)
{
	this->_state = initial_satate;
	this->_covariance = initial_covariance;
	this->_transition_matrix = transition_matrix;
	this->_observation_matrix = observation_matrix;
	this->_process_noise_covariance = process_noise_covariance;
	this->_measurement_noise_covariance = measurement_noise_covariance;
}

template <typename K>
KalmanFilter<K>::KalmanFilter(const KalmanFilter &kalmanfilter)
{
	*this = kalmanfilter;
}

template <typename K>
KalmanFilter<K>	&KalmanFilter<K>::operator=(const KalmanFilter &kalmanfilter)
{
	if (this == &kalmanfilter)
		return (*this);
	this->_state = kalmanfilter._state;
	this->_covariance = kalmanfilter._covariance;
	this->_transition_matrix = kalmanfilter._transition_matrix;
	this->_observation_matrix = kalmanfilter._observation_matrix;
	this->_process_noise_covariance = kalmanfilter._process_noise_covariance;
	this->_measurement_noise_covariance = kalmanfilter._measurement_noise_covariance;
	return (*this);
}

template <typename K>
KalmanFilter<K>::~KalmanFilter()
{
}

template <typename K>
Vector<K>	KalmanFilter<K>::getState(void) const
{
	return (this->_state);
}

/* - predicted state x̂ₖ = Fₖ * x̂ₖ₋₁()
   - predicted covariance Pₖ = Fₖ * Pₖ₋₁ * Fₖᵀ + Qₖ
   (F: transition matrix, Q: process noise covariance matrix) */
template <typename K>
void	KalmanFilter<K>::predict(void)
{
	this->_state = this->_transition_matrix.mul_vec(this->_state);
	Matrix<K>	tmp = this->_transition_matrix.mul_mat(this->_covariance).mul_mat(this->_transition_matrix.transpose());
	tmp.add(_process_noise_covariance);
	this->_covariance = tmp;
}

/* - innovation ỹₖ = zₖ - Hₖ * x̂ₖ
   - innovation covariance Sₖ = Hₖ * Pₖ * Hₖᵀ + Rₖ 
   - kalman gain Kₖ = Pₖ * Hₖᵀ * Sₖ⁻¹
   (H: observation matrix, z: actual measurement, R: measurement noise covariance matrix)
   - updated estimated state x̂ₖ = x̂ₖ + Kₖ * ỹₖ
   - updated estimated covariance Pₖ = (I - Kₖ * Hₖ) * Pₖ */
template <typename K>
void	KalmanFilter<K>::update(Vector<K> measurement)
{
	Vector<K>	tmp_vec = measurement;
	tmp_vec.sub(this->_observation_matrix.mul_vec(this->_state));
	Vector<K>	innovation = tmp_vec;
	Matrix<K>	tmp = this->_observation_matrix.mul_mat(this->_covariance).mul_mat(this->_observation_matrix.transpose());
	tmp.add(this->_measurement_noise_covariance);
	Matrix<K>	innovation_covariance = tmp;
	Matrix<K>	kalman_gain = this->_covariance.mul_mat(this->_observation_matrix.transpose()).mul_mat(innovation_covariance.inverse());

	tmp_vec = this->_state;
	tmp_vec.add(kalman_gain.mul_vec(innovation));
	this->_state = tmp_vec;
	tmp = this->_covariance.identity();
	tmp.sub(kalman_gain.mul_mat(this->_observation_matrix));
	this->_covariance = tmp.mul_mat(this->_covariance);
}
