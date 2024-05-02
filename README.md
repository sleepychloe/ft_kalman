currently working on the project

## Installation
```
  https://github.com/sleepychloe/ft_kalman.git
  cd ft_kalman/ft_kalman
  make
```

## Usage
```
  ./imu-sensor-stream-linux -s 42 -d 42 -p 4242
  ./ft_kalman
```

## Kalman filter

### Class Template KalmanFilter<K>
#### predict
 ⋅ predicted state x̂ₖ = Fₖ * x̂ₖ₋₁
 ⋅ predicted covariance Pₖ = Fₖ * Pₖ₋₁ * Fₖᵀ + Qₖ
&nbsp;&nbsp;&nbsp;&nbsp;(F: transition matrix, Q: process noise covariance matrix)

```
  template <typename K>
  void	KalmanFilter<K>::predict(void)
  {
      this->_state = this->_transition_matrix * this->_state;
    	this->_covariance = this->_transition_matrix
                            * this->_covariance
                            * this->_transition_matrix.transpose()
                            + _process_noise_covariance;
  }
```

#### update
 ⋅ innovation ỹₖ = zₖ - Hₖ * x̂ₖ
 ⋅ innovation covariance Sₖ = Hₖ * Pₖ * Hₖᵀ + Rₖ
 ⋅ kalman gain Kₖ = Pₖ * Hₖᵀ * Sₖ⁻¹
&nbsp;&nbsp;&nbsp;&nbsp;(H: observation matrix, z: actual measurement, R: measurement noise covariance matrix)
 ⋅ updated estimated state x̂ₖ = x̂ₖ + Kₖ * ỹₖ
 ⋅ updated estimated covariance Pₖ = (I - Kₖ * Hₖ) * Pₖ
```
  template <typename K>
  void	KalmanFilter<K>::update(Vector<K> measurement)
  {
  	  Vector<K>	innovation = measurement - this->_observation_matrix * this->_state;
  	  Matrix<K>	innovation_covariance = this->_observation_matrix
                                          * this->_covariance
                                          * this->_observation_matrix.transpose()
  					                            	+ this->_measurement_noise_covariance;
  	  Matrix<K>	kalman_gain = this->_covariance
                                * this->_observation_matrix.transpose()
                                * innovation_covariance.inverse();
  	this->_state = this->_state + kalman_gain * innovation;
  	this->_covariance = (identity<double>(this->_state.getSize())- kalman_gain * _observation_matrix)
  				                * this->_covariance;
  }
```

![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/019a912f-8248-4744-a2c6-7ce9951eb3cc)
![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/a7a1838c-c44b-4e67-aeb1-7b1f40c7b5ca)
