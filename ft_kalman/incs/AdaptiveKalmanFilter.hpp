/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   AdaptiveKalmanFilter.hpp                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/06/14 01:56:58 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/14 22:15:14 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef ADAPTIVE_KALMAN_FILTER_HPP
# define ADAPTIVE_KALMAN_FILTER_HPP

#include "./KalmanFilter.hpp"

template <typename K>
class AdaptiveKalmanFilter: public KalmanFilter
{
public:
	AdaptiveKalmanFilter();
	AdaptiveKalmanFilter(Vector<K> initial_state, Matrix<K> initial_covariance,
			Matrix<K> transition_matrix, Matrix<K> observation_matrix,
			Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance);
	AdaptiveKalmanFilter(Vector<K> initial_state, Matrix<K> initial_covariance,
			Matrix<K> transition_matrix, Matrix<K> control_transition_matrix, Matrix<K> observation_matrix,
			Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance);
	AdaptiveKalmanFilter(const AdaptiveKalmanFilter &adaptivekalmanfilter);
	AdaptiveKalmanFilter& operator=(const AdaptiveKalmanFilter &adaptivekalmanfilter);
	~AdaptiveKalmanFilter();

	virtual void		update(Vector<K> measurement);

private:
	void			adaptNoiseCovariance(void);

	K			_adaptation_rate;
};

#include "../srcs/AdaptiveKalmanFilter.cpp"

#endif
