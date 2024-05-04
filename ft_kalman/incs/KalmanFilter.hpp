/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   KalmanFilter.hpp                                   :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/25 01:15:47 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/04 16:33:51 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef KALMAN_FILTER_HPP
# define KALMAN_FILTER_HPP

#include "../matrix/incs/Vector.hpp"
#include "../matrix/incs/Matrix.hpp"

template <typename K>
class KalmanFilter
{
public:
	KalmanFilter();
	KalmanFilter(Vector<K> initial_state, Matrix<K> initial_covariance,
			Matrix<K> transition_matrix, Matrix<K> observation_matrix,
			Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance);
	KalmanFilter(Vector<K> initial_state, Matrix<K> initial_covariance,
			Matrix<K> transition_matrix, Matrix<K> control_transition_matrix, Matrix<K> observation_matrix,
			Matrix<K> process_noise_covariance, Matrix<K> measurement_noise_covariance);
	KalmanFilter(const KalmanFilter &kalmanfilter);
	KalmanFilter& operator=(const KalmanFilter &KalmanFilter);
	~KalmanFilter();

	Vector<K>		getState(void) const;

	void			predict(void);
	void			predict(Vector<K> control_input);
	void			update(Vector<K> measurement);

private:
	Vector<K>		_state;
	Matrix<K>		_covariance;
	Matrix<K>		_transition_matrix;
	Matrix<K>		_control_transition_model;
	Matrix<K>		_observation_matrix;
	Matrix<K>		_process_noise_covariance;
	Matrix<K>		_measurement_noise_covariance;
};

#include "../srcs/KalmanFilter.cpp"

#endif
