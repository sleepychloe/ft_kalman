/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   AdaptiveKalmanFilter.cpp                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/06/14 21:55:02 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/14 22:23:46 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/AdaptiveKalmanFilter.hpp"

template <typename K>
AdaptiveKalmanFilter<K>::AdaptiveKalmanFilter(): KalmanFilter()
{
}

template <typename K>
AdaptiveKalmanFilter<K>::AdaptiveKalmanFilter(Vector<K> initial_state, Matrix<K> initial_covariance,
			Matrix<K> transition_matrix, Matrix<K> observation_matrix,
			Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance)
				:KalmanFilter(initial_state, initial_covariance,
						transition_matrix, observation_matrix,
						process_noise_covariance, measurement_noise_covariance)

{
	this->_adaptation_rate = 0.1;
}

template <typename K>
AdaptiveKalmanFilter<K>::AdaptiveKalmanFilter(Vector<K> initial_state, Matrix<K> initial_covariance,
			Matrix<K> transition_matrix, Matrix<K> control_transition_matrix, Matrix<K> observation_matrix,
			Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance)
				:KalmanFilter(initial_state, initial_covariance,
						transition_matrix, control_transition_matrix, observation_matrix,
						process_noise_covariance, measurement_noise_covariance)
{
	this->_adaptation_rate = 0.1;
}

template <typename K>
AdaptiveKalmanFilter<K>::AdaptiveKalmanFilter(const AdaptiveKalmanFilter &adaptivekalmanfilter): KalmanFilter(adaptivekalmanfilter)
{
	*this = adaptivekalmanfilter;
}

template <typename K>
AdaptiveKalmanFilter<K> &AdaptiveKalmanFilter<K>::operator=(const AdaptiveKalmanFilter &adaptivekalmanfilter)
{
	if (this == &adaptivekalmanfilter)
		return (*this);
	this->_state = adaptivekalmanfilter._state;
	this->_covariance = adaptivekalmanfilter._covariance;
	this->_transition_matrix = adaptivekalmanfilter._transition_matrix;
	this->_control_transition_model = adaptivekalmanfilter._control_transition_model;
	this->_observation_matrix = adaptivekalmanfilter._observation_matrix;
	this->_process_noise_covariance = adaptivekalmanfilter._process_noise_covariance;
	this->_measurement_noise_covariance = adaptivekalmanfilter._measurement_noise_covariance;
	this->_adaptation_rate = adaptivekalmanfilter._adaptation_rate;
	return (*this);
}

template <typename K>
AdaptiveKalmanFilter<K>::~AdaptiveKalmanFilter()
{
}

template <typename K>
void	AdaptiveKalmanFilter<K>::update(Vector<K> measurement)
{
	KalmanFilter<K>::update(measurement);
	adaptNoiseCovariance();
}

template <typename K>
void	AdaptiveKalmanFilter<K>::adaptNoiseCovariance(void)
{
	Vector<K>	innovation = measurement - this->_observation_matrix * this->_state;
	Matrix<K>	state_matrix(1, this->_state.getSize());
	for (size_t i = 0; i < this->_state.getSize(); i++)
		state_matrix[0][i] = this->_state.getVector()[i];

	this->_measurement_noise_covariance = (1 - this->_adaptation_rate) * this->_measurement_noise_covariance
						+ this->_adaptation_rate * (innovation * innovation.transpose());
	this->_process_noise_covariance = (1 - this->_adaptation_rate) * this->_process_noise_covariance
						+ this->_adaptation_rate * (state_matrix * state_matrix.transpose());
}
