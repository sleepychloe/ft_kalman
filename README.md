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
 ⋅ predicted state x̂ₖ = Fₖ * x̂ₖ₋₁<br>
 ⋅ predicted covariance Pₖ = Fₖ * Pₖ₋₁ * Fₖᵀ + Qₖ<br>
&nbsp;&nbsp;(F: transition matrix, Q: process noise covariance matrix)

```
  template <typename K>
  void	KalmanFilter<K>::predict(void)
  {
	this->_state
		= this->_transition_matrix * this->_state;
	this->_covariance = this->_transition_matrix * this->_covariance * this->_transition_matrix.transpose()
				+ _process_noise_covariance;
  }
```
<br>
#### update
 ⋅ innovation ỹₖ = zₖ - Hₖ * x̂ₖ<br>
 ⋅ innovation covariance Sₖ = Hₖ * Pₖ * Hₖᵀ + Rₖ<br>
 ⋅ kalman gain Kₖ = Pₖ * Hₖᵀ * Sₖ⁻¹<br>
&nbsp;&nbsp;(H: observation matrix, z: actual measurement, R: measurement noise covariance matrix)<br>
 ⋅ updated estimated state x̂ₖ = x̂ₖ + Kₖ * ỹₖ<br>
 ⋅ updated estimated covariance Pₖ = (I - Kₖ * Hₖ) * Pₖ<br>
```
  template <typename K>
  void	KalmanFilter<K>::update(Vector<K> measurement)
  {
	Vector<K>	innovation = measurement - this->_observation_matrix * this->_state;
	Matrix<K>	innovation_covariance = this->_observation_matrix * this->_covariance * this->_observation_matrix.transpose()
						+ this->_measurement_noise_covariance;
	Matrix<K>	kalman_gain = this->_covariance * this->_observation_matrix.transpose() * innovation_covariance.inverse();
	this->_state = this->_state + kalman_gain * innovation;
	this->_covariance = (identity<double>(this->_state.getSize()) - kalman_gain * _observation_matrix)
				* this->_covariance;
  }
```
<br>
### How to calculate
After sending "READY" to server, you can get<br>
&nbsp;&nbsp;⋅ true initial position(x, y, z in meters)<br>
&nbsp;&nbsp;⋅ true initial speed(in km/h)<br>
&nbsp;&nbsp;⋅ current acceleration(in m/s²) every 0.01 second<br>
&nbsp;&nbsp;⋅ current direction(in euler angles) every 0.01 second.<br>
<br>
If you send correct estimated position to server, you can also get<br>
&nbsp;&nbsp;⋅ current GPS position(in meters) every 3 seconds<br>
<br>
Subject said that vehicle moves only on its longitudinal axis (in front of it),<br>
which means the vehicle moves towards +x, +y, or +z aixs.<br>
Thus, the true initial velocity of the vehicle in m/s will be one of three cases:<br>
&nbsp;&nbsp;1. v(true initial speed in m/s, 0, 0)<br>
&nbsp;&nbsp;2. v(0, true initial speed in m/s, 0)<br>
&nbsp;&nbsp;3. (0, 0, true initial speed in m/s)<br>
(try it from the first case - the actual velocity was (true initial speed in m/s, 0, 0))
<br>
Even though server doesn't give you vehicle's velocity directly, you can calculate it from direction and acceleration.<br>
```
	┏		      ┓
	┃   1      0      0   ┃
Rx(ψ) = ┃   0     cosψ  −sinψ ┃
	┃   0     sinψ   cosψ ┃
	┗		      ┛
	┏		      ┓
	┃  cosθ    0     sinθ ┃
Ry(θ) = ┃   0      1      0   ┃
	┃ −sinθ    0     cosθ ┃
	┗		      ┛
	┏		      ┓
	┃  cosφ  −sinφ    0   ┃
Rz(φ) = ┃  sinφ   cosφ    0   ┃
	┃   0      0      1   ┃
	┗		      ┛
R = Rz(φ)Ry(θ)Rx(ψ)
 ₂
 ∑ global_a[k] = r[k][i] * a[i] (k = x, y, z)
ⁱ⁼⁰
v[k] = v[k] + global_a[k] *  ∆t
```
<br>
Now you have initial position, initial velocity, and acceleration,<br>
and you can calculate the position after 0.01 second(=∆t) with Newton's laws of motion.<br>
<br>
Compute until the GPS position is received from the server(Kalman filter predict)<br>
You can compare the calculation result and the actual position every 3 seconds(Kalman filter update)<br>.
<br>
### How to initialize Kalman filter
<br>
<br>
![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/019a912f-8248-4744-a2c6-7ce9951eb3cc)
![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/a7a1838c-c44b-4e67-aeb1-7b1f40c7b5ca)
