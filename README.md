currently working on the project

## Installation
```
  git clone https://github.com/sleepychloe/ft_kalman.git
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
&nbsp;&nbsp;(F: transition matrix,<br>
&nbsp;&nbsp;&nbsp;Q: process noise covariance matrix)<br>

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
#### update
 ⋅ innovation ỹₖ = zₖ - Hₖ * x̂ₖ<br>
 ⋅ innovation covariance Sₖ = Hₖ * Pₖ * Hₖᵀ + Rₖ<br>
 ⋅ kalman gain Kₖ = Pₖ * Hₖᵀ * Sₖ⁻¹<br>
&nbsp;&nbsp;(H: observation matrix,<br>
&nbsp;&nbsp;&nbsp;z: actual measurement<br>
&nbsp;&nbsp;&nbsp;R: measurement noise covariance matrix)<br>
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
<br>
Thus, the true initial velocity of the vehicle in m/s will be one of three cases:<br>
&nbsp;&nbsp;1. v(true initial speed in m/s, 0, 0)<br>
&nbsp;&nbsp;2. v(0, true initial speed in m/s, 0)<br>
&nbsp;&nbsp;3. (0, 0, true initial speed in m/s)<br>
(try it from the first case<br>
&nbsp;&nbsp;&nbsp;&nbsp;- the actual velocity was (true initial speed in m/s, 0, 0) when I tried)<br>
<br>
Even though server does not give us vehicle's velocity directly,<br>
you can calculate it from direction and acceleration.<br>
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
v[k] = v[k] + global_a[k] * ∆t
```
Now you have initial position, initial velocity, and acceleration,<br>
and you can calculate the position after 0.01 second(=∆t) with Newton's laws of motion.<br>
<br>
Compute until the GPS position is received from the server(Kalman filter predict),<br>
and you can compare the calculation result and the actual position every 3 seconds(Kalman filter update).<br>
<br>
### How to initialize Kalman filter
⋅ predicted state x̂ₖ = Fₖ * x̂ₖ₋₁<br>
The position, velocity, and acceleration of the vehicle are described by the linear state space.<br>
Thus, vector x̂ₖ can be defined as (kₖ(=position), k̇ₖ(=velocity), k̈ₖ(=acceleration)) (k = x, y, z).<br><br>
By Newton's laws of motion,<br>
kₖ = kₖ₋₁ + k̇ₖ₋₁∆t + k̈ₖ₋₁∆t²/2<br>
k̇ₖ = k̇ₖ₋₁ + k̈ₖ₋₁∆t<br>
k̈ₖ = k̈ₖ₋₁<br>
```
x̂ₖ = (kₖ, k̇ₖ, k̈ₖ)
    ┏                      ┓
    ┃   1      ∆t   0.5∆t² ┃
F = ┃   0      1     ∆t    ┃
    ┃   0      0      1    ┃
    ┗                      ┛
```
⋅ predicted covariance Pₖ = Fₖ * Pₖ₋₁ * Fₖᵀ + Qₖ<br>
We already know the init state of the vehicle.<br>
Make diagonal matrix with GPS, gyroscope, and accelerometer noise<br>
instead of using appropriately large value.<br>

```
    ┏                      ┓
    ┃   σ²ₚ    0      0    ┃
P = ┃   0      σ²ᵥ    0    ┃
    ┃   0      0      σ²ₐ  ┃
    ┗                      ┛
(σ²ᵥ = σ_gyroscope² + σ_accelerometer² * ∆t)
```

This is kinematic system, and it is continuous.<br>
So you can apply continuous white noise model for Q.<br>
<br>
FQcFᵀ is a projection of the continuous noise based on F.<br>
Since the noise is changing continuously, and we want to know how much noise is added to the system over the interval <span>[0, ∆t]</span>,<br>
you need to integrate FQcFᵀ.<br>

```
Q =  ∫₀ΔᵗFQcFᵀ
(Qc: continuous noise)
```
when I tried, filter worked as expected with following matrix Qc:<br>
```
     ┏                 ┓ ┏                            ┓
     ┃ ∆t²/2   0     0 ┃ ┃ σ²ₚ∆t³/6     0         0   ┃
Qc = ┃   0     ∆t    0 ┃*┃    0      σ²ᵥ∆t²/2     0   ┃
     ┃   0     0     1 ┃ ┃    0         0       σ²ₐ∆t ┃
     ┗                 ┛ ┗                            ┛
```
To express definite integral of the expression I used Riemann sum method:<br>
```
	    ₙ
∫ₐᵇf(x)dx ≈ ∑f(a + i*(b - a)/n) * (b-a)/n
	   ⁱ⁼¹
```
```
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
```
<br>
⋅ innovation ỹₖ = zₖ - Hₖ * x̂ₖ<br>
⋅ innovation covariance Sₖ = Hₖ * Pₖ * Hₖᵀ + Rₖ<br>
When GPS position is received from server, you can update the filter.<br>
I did not use velocity information to update kalman filter,<br>
because I wanted to use raw data which I do not have to compute.<br>

```
zₖ = (kₖ, k̈̈ₖ)
    ┏           ┓
    ┃ 1   0   0 ┃
H = ┃ 0   0   1 ┃
    ┗           ┛
    ┏          ┓
    ┃ σ²ₚ   0  ┃
R = ┃  0   σ²ₐ ┃
    ┗          ┛
```
<br>
<br>

![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/019a912f-8248-4744-a2c6-7ce9951eb3cc)
![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/a7a1838c-c44b-4e67-aeb1-7b1f40c7b5ca)
