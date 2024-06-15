/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   AdaptiveKalmanFilter.cpp                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/06/14 21:55:02 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/15 17:35:43 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/AdaptiveKalmanFilter.hpp"

template <typename K>
AdaptiveKalmanFilter<K>::AdaptiveKalmanFilter(): KalmanFilter<K>()
{
}

template <typename K>
AdaptiveKalmanFilter<K>::AdaptiveKalmanFilter(Vector<K> initial_state, Matrix<K> initial_covariance,
			Matrix<K> transition_matrix, Matrix<K> observation_matrix,
			Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance)
				:KalmanFilter<K>(initial_state, initial_covariance,
						transition_matrix, observation_matrix,
						process_noise_covariance, measurement_noise_covariance)

{
	this->_adaptation_rate = 0.0000003;
}

template <typename K>
AdaptiveKalmanFilter<K>::AdaptiveKalmanFilter(Vector<K> initial_state, Matrix<K> initial_covariance,
			Matrix<K> transition_matrix, Matrix<K> control_transition_matrix, Matrix<K> observation_matrix,
			Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance)
				:KalmanFilter<K>(initial_state, initial_covariance,
						transition_matrix, control_transition_matrix, observation_matrix,
						process_noise_covariance, measurement_noise_covariance)
{
	this->_adaptation_rate = 0.0000003;
}

template <typename K>
AdaptiveKalmanFilter<K>::AdaptiveKalmanFilter(const AdaptiveKalmanFilter &adaptivekalmanfilter)
				: KalmanFilter<K>(adaptivekalmanfilter)
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
	Vector<K>	innovation = measurement - this->_observation_matrix * this->_state;
	Matrix<K>	innovation_covariance = this->_observation_matrix * this->_covariance * this->_observation_matrix.transpose()
						+ this->_measurement_noise_covariance;
	Matrix<K>	kalman_gain = this->_covariance * this->_observation_matrix.transpose() * innovation_covariance.inverse();

	this->_state = this->_state + kalman_gain * innovation;
	this->_covariance = (identity<double>(this->_state.getSize()) - kalman_gain * this->_observation_matrix)
				* this->_covariance;
	adaptNoiseCovariance(innovation_covariance);
}

/* - adaptive measurement noise covariance Rₖ = (1 − α) * Rₖ + α * Sₖ
   - adaptive process noise covariance Qₖ = (1 - α) * Qₖ + α * Pₖ
   (α: adaptation rate,
    R: measurement noise covariance matrix,
    Q: process noise covariance)
*/
template <typename K>
void	AdaptiveKalmanFilter<K>::adaptNoiseCovariance(Matrix<K> innovation_covariance)
{
	this->_measurement_noise_covariance = (1 - this->_adaptation_rate) * this->_measurement_noise_covariance
						+ this->_adaptation_rate * innovation_covariance;
	this->_process_noise_covariance = (1 - this->_adaptation_rate) * this->_process_noise_covariance
						+ this->_adaptation_rate * this->_covariance;
}
