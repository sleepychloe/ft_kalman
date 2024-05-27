currently working on the project

## Installation & Usage

```
  git clone https://github.com/sleepychloe/ft_kalman.git
  cd ft_kalman
  make
```
If xorg is not installed, install it via
```
  sudo apt-get install -y xorg-dev
```
If docker and docker-compose is not isntalled, install it via
```
  sudo apt install -y docker.io && sudo apt-get install -y docker-compose
```
<br>

Before running program, allow the root user on your local system to access the X server via
```
  xhost +local:root
```
To run server:
```
  docker exec -it kalman /bin/bash
  ./imu-sensor-stream-linux -s 42 -d 42 -p 4242
```
To run program:
```
  docker exec -it kalman /bin/bash
  make
  ./ft_kalman
```
use --graph to see the graph.
```
  ./ft_kalman --graph
```
When you are done testing, do not forget to disable root access via
```
  xhost -local:root
```
<br>

<details>
<summary><b><ins>launch without docker: when you are a root user</ins></b></summary>

If you are a root user and do not want to use docker,<br>
install softwares that use OpenGL and related libraries for rendering graphics via
```
  sudo apt-get install -y build-essential \
			  xorg-dev \
			  libglu1-mesa-dev \
			  mesa-common-dev \
			  libglew-dev \
			  libglfw3-dev \
			  libglm-dev
```
Clone the repository and compile
```
  git clone https://github.com/sleepychloe/ft_kalman.git
  cd ft_kalman/ft_kalman
  make
```
To run server:
```
  ./imu-sensor-stream-linux -s 42 -d 42 -p 4242
```
To run program:
```
  ./ft_kalman
```
use --graph to see the graph.
```
  ./ft_kalman --graph
```
</details>
<br>

## Kalman filter

### Class Template KalmanFilter<K>
#### predict without control input
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
#### predict with control input
I introduced control input for this project, to calculate the effect of external inputs(=acceleration).<br>
 ⋅ predicted state x̂ₖ = Fₖ * x̂ₖ₋₁ + B * uₖ<br>
 ⋅ predicted covariance Pₖ = Fₖ * Pₖ₋₁ * Fₖᵀ + Qₖ<br>
&nbsp;&nbsp;(F: transition matrix,<br>
&nbsp;&nbsp;&nbsp;B: control transition matrix,<br>
&nbsp;&nbsp;&nbsp;u: control input,<br>
&nbsp;&nbsp;&nbsp;Q: process noise covariance matrix)<br>
```
template <typename K>
void	KalmanFilter<K>::predict(Vector<K> control_input)
{
	this->_state
		= this->_transition_matrix * this->_state + this->_control_transition_model * control_input;
	this->_covariance = this->_transition_matrix * this->_covariance * this->_transition_matrix.transpose()
				+ this->_process_noise_covariance;
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
<br>
### How to initialize Kalman filter
⋅ predicted state x̂ₖ = Fₖ * x̂ₖ₋₁ + B * uₖ<br>
The position, and velocity of the vehicle are described by the linear state space.<br>
Thus, vector x̂ₖ can be defined as (kₖ(=position), k̇ₖ(=velocity)) (k = x, y, z).<br><br>
By Newton's laws of motion,<br>
kₖ = kₖ₋₁ + k̇ₖ₋₁∆t + k̈ₖ₋₁∆t²/2<br>
k̇ₖ = k̇ₖ₋₁ + k̈ₖ₋₁∆t<br>
By definition of velocity,
v = Δd / Δt<br>
```
x̂ₖ = (kₖ, k̇ₖ)
    ┏          ┓
F = ┃ 1     ∆t ┃
    ┃ 0     1  ┃
    ┗          ┛
```
The acceleration can be considered as an external inputs.<br>
Thus vecter u can be defined as (k̈ₖ) (k = x, y, z).<br>
Let's say that the acceleration is constant during the interval of time ∆t.<br>
By Newton's laws of motion,<br>
k̈ₖ = k̈ₖ₋₁<br>
By definition of acceleration,
a = Δv / Δt<br>
```
u = (k̈ₖ)
    ┏        ┓
B = ┃ 0.5∆t² ┃
    ┃   ∆t   ┃
    ┗        ┛
```
<details>
<summary><b><ins>see another way to calculate: define x̂ₖ as (kₖ, k̇ₖ, k̈ₖ)</ins></b></summary>
The position, velocity, and acceleration of the vehicle are described by the linear state space.
You can also define vector x̂ₖ as (kₖ(=position), k̇ₖ(=velocity), k̈ₖ(=acceleration)) (k = x, y, z).<br>

```
x̂ₖ = (kₖ, k̇ₖ, k̈ₖ)
    ┏                      ┓
    ┃   1      ∆t   0.5∆t² ┃
F = ┃   0      1     ∆t    ┃
    ┃   0      0      1    ┃
    ┗                      ┛
```

Setting control input u and control transition model is optional in this case.<br>
</details>
<br>

⋅ predicted covariance Pₖ = Fₖ * Pₖ₋₁ * Fₖᵀ + Qₖ<br>
We already know the init state of the vehicle.<br>
Make diagonal matrix with GPS, and gyroscope<br>
instead of using appropriately large value.<br>

```
    ┏           ┓
P = ┃ σ²ₚ    0  ┃
    ┃  0    σ²ᵥ ┃
    ┗           ┛
(σ²ᵥ = σ_gyroscope² + σ_accelerometer² * ∆t)
```
<details>
<summary><b><ins>when x̂ₖ is (kₖ, k̇ₖ, k̈ₖ)</ins></b></summary>
You need to apply accelerometer noise for matrix P.<br>

```
    ┏                 ┓
    ┃ σ²ₚ   0     0   ┃
P = ┃ 0     σ²ᵥ   0   ┃
    ┃ 0     0     σ²ₐ ┃
    ┗                 ┛
(σ²ᵥ = σ_gyroscope² + σ_accelerometer² * ∆t)
```
</details>
<br>

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
     ┏               ┓
Qc = ┃ ∆t²/2   ∆t²/2 ┃*Φₛ
     ┃   0       ∆t  ┃
     ┗               ┛
(Φₛ: spectral density of the white noise, Φₛ = Nₚ + Nᵥ + Nₐ)
     ┏            ┓
Nₚ = ┃  σ²ₚ    0  ┃
     ┃  0      0  ┃
     ┗            ┛
     ┏            ┓
Nᵥ = ┃  0   σ²ᵥ∆t ┃
     ┃  0    σ²ᵥ  ┃
     ┗            ┛
     ┏                    ┓
Nₐ = ┃ σ²ₐ∆t²/2  σ²ₐ∆t²/2 ┃
     ┃    0        σ²ₐ∆t  ┃
     ┗                    ┛
```
Φs is multiplied to the term that represents acceleration.<br>
We assume that the system has constant acceleration,<br>
and the noise affects the acceleration.<br>

<details>
<summary><b><ins>when x̂ₖ is (kₖ, k̇ₖ, k̈ₖ)</ins></b></summary>
When I tried, filter worked as expected with following matrix Qc:<br>

```
     ┏                 ┓
     ┃ ∆t²/2   0     0 ┃
Qc = ┃   0     ∆t    0 ┃*Φₛ
     ┃   0     0     1 ┃
     ┗                 ┛
(Φₛ = Nₚ + Nᵥ + Nₐ)
     ┏                 ┓
     ┃   σ²ₚ   0     0 ┃
Nₚ = ┃   0     0     0 ┃
     ┃   0     0     0 ┃
     ┗                 ┛
     ┏                 ┓
     ┃ σ²ᵥ∆t   0     0 ┃
Nᵥ = ┃   0    σ²ᵥ    0 ┃
     ┃   0     0     0 ┃
     ┗                 ┛
     ┏                       ┓
     ┃ σ²ₐ∆t²/2   0       0  ┃
Nₐ = ┃   0      σ²ₐ∆t     0  ┃
     ┃   0        0      σ²ₐ ┃
     ┗                       ┛
```
</details>
<br>

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
because I wanted to use raw data which I do not have to compute.<br><br>

```
zₖ = (kₖ)
    ┏       ┓
H = ┃ 1   0 ┃
    ┗       ┛
    ┏     ┓
R = ┃ σ²ₚ ┃
    ┗     ┛
```
<details>
<summary><b><ins>when x̂ₖ is (kₖ, k̇ₖ, k̈ₖ)</ins></b></summary>
When I tried, filter worked as expected with following matrix Qc:<br>

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
</details>
<br>

### initial values
```
/* control input: n */
control_input = Vector<double>({p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
kalman.predict(control_input);

/* measurement: m */
measurement = Vector<double>({p.getPos()[0], p.getPos()[1], p.getPos()[2]});
kalman.update(measurement);
```

```
/* init state: n(pos, v) */
Vector<double>	init_state({p.getPos()[0], p.getPos()[1], p.getPos()[2], v[0], v[1], v[2]});

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
Matrix<double>	q_continuous({{DT * DT / 2, 0, 0, DT * DT / 2, 0, 0},
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
Matrix<double>	noise_density = noise_p + noise_v + noise_a;
q_continuous = q_continuous * noise_density;
Matrix<double>	process_noise_covariance = transition_matrix * q_continuous * transition_matrix.transpose();
process_noise_covariance = integrate(process_noise_covariance, 0, DT);

/* measurement noise: m by m */
Matrix<double>	measurement_noise_covariance({{variance_p, 0, 0},
					{0, variance_p, 0},
					{0, 0, variance_p}});

kalman = KalmanFilter<double>(init_state, init_covariance,
				transition_matrix, observation_matrix, control_transition_model,
				process_noise_covariance, measurement_noise_covariance);
```
<details>
<summary><b><ins>when x̂ₖ is (kₖ, k̇ₖ, k̈ₖ)</ins></b></summary>

```
/* without control input */
kalman.predict();

/* measurement: m */
measurement = Vector<double>({p.getPos()[0], p.getPos()[1], p.getPos()[2], p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
kalman.update(measurement);
```

```
/* init state: n(pos, v, acc) */
Vector<double>	init_state({p.getPos()[0], p.getPos()[1], p.getPos()[2], v[0], v[1], v[2], p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});

/* init covariance: n by n */
Matrix<double>	init_covariance({{pow(GPS_NOISE, 2), 0, 0, 0, 0, 0, 0, 0, 0},
				{0, pow(GPS_NOISE, 2), 0, 0, 0, 0, 0, 0, 0},
				{0, 0, pow(GPS_NOISE, 2), 0, 0, 0, 0, 0, 0},
				{0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT), 0, 0, 0, 0, 0},
				{0, 0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT), 0, 0, 0, 0},
				{0, 0, 0, 0, 0, (pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT), 0, 0, 0},
				{0, 0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE, 2), 0, 0},
				{0, 0, 0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE, 2), 0},
				{0, 0, 0, 0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE, 2)}});

/* transition: n by n */
Matrix<double>	transition_matrix({{1, 0, 0, DT, 0, 0, DT * DT / 2, 0, 0},
					{0, 1, 0, 0, DT, 0, 0, DT * DT / 2, 0},
					{0, 0, 1, 0, 0, DT, 0, 0, DT * DT / 2},
					{0, 0, 0, 1, 0, 0, DT, 0, 0},
					{0, 0, 0, 0, 1, 0, 0, DT, 0},
					{0, 0, 0, 0, 0, 1, 0, 0, DT},
					{0, 0, 0, 0, 0, 0, 1, 0, 0},
					{0, 0, 0, 0, 0, 0, 0, 1, 0},
					{0, 0, 0, 0, 0, 0, 0, 0, 1}});
/* observation: m by n */
Matrix<double>	observation_matrix({{1, 0, 0, 0, 0, 0, 0, 0, 0},
					{0, 1, 0, 0, 0, 0, 0, 0, 0},
					{0, 0, 1, 0, 0, 0, 0, 0, 0},
					{0, 0, 0, 0, 0, 0, 1, 0, 0},
					{0, 0, 0, 0, 0, 0, 0, 1, 0},
					{0, 0, 0, 0, 0, 0, 0, 0, 1}});

/* process_noise: n by n */
Matrix<double>	q_continuous({{DT * DT / 2, 0, 0, 0, 0, 0, 0, 0, 0,},
				{0, DT * DT / 2, 0, 0, 0, 0, 0, 0, 0,},
				{0, 0, DT * DT / 2, 0, 0, 0, 0, 0, 0,},
				{0, 0, 0, DT, 0, 0, 0, 0, 0,},
				{0, 0, 0, 0, DT, 0, 0, 0, 0,},
				{0, 0, 0, 0, 0, DT, 0, 0, 0,},
				{0, 0, 0, 0, 0, 0, 1, 0, 0,},
				{0, 0, 0, 0, 0, 0, 0, 1, 0,},
				{0, 0, 0, 0, 0, 0, 0, 0, 1,}});
Matrix<double>	noise_p({{pow(GPS_NOISE, 2), 0, 0, 0, 0, 0, 0, 0, 0},
				{0, pow(GPS_NOISE, 2), 0, 0, 0, 0, 0, 0, 0},
				{0, 0, pow(GPS_NOISE, 2), 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},});
double		variance_p = pow(GPS_NOISE, 2);
double		variance_v = pow(GYROSCOPE_NOISE, 2) + pow(ACCELEROMETER_NOISE, 2) * DT;
double		variance_a = pow(ACCELEROMETER_NOISE, 2);
Matrix<double>	noise_p({{variance_p, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, variance_p, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, variance_p, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0}});
Matrix<double>	noise_v({{variance_v * DT, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, variance_v * DT, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, variance_v * DT, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, variance_v, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, variance_v, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, variance_v, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, 0}});
Matrix<double>	noise_a({{variance_a * DT * DT / 2, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, variance_a * DT * DT / 2, 0, 0, 0, 0, 0, 0, 0},
				{0, 0, variance_a * DT * DT / 2, 0, 0, 0, 0, 0, 0},
				{0, 0, 0, variance_a * DT, 0, 0, 0, 0, 0},
				{0, 0, 0, 0, variance_a * DT, 0, 0, 0, 0},
				{0, 0, 0, 0, 0, variance_a * DT, 0, 0, 0},
				{0, 0, 0, 0, 0, 0, variance_a, 0, 0},
				{0, 0, 0, 0, 0, 0, 0, variance_a, 0},
				{0, 0, 0, 0, 0, 0, 0, 0, variance_a}});
Matrix<double>	noise_density = noise_p + noise_v + noise_a;
q_continuous = q_continuous * noise_density ;
Matrix<double>	process_noise_covariance = transition_matrix * q_continuous * transition_matrix.transpose();
process_noise_covariance = integrate(process_noise_covariance, 0, DT);

/* measurement noise: m by m */
Matrix<double>	measurement_noise_covariance({{variance_p, 0, 0, 0, 0, 0},
					{0, variance_p, 0, 0, 0, 0},
					{0, 0, variance_p, 0, 0, 0},
					{0, 0, 0, variance_a, 0, 0},
					{0, 0, 0, 0, variance_a, 0},
					{0, 0, 0, 0, 0, variance_a}});

kalman = KalmanFilter<double>(init_state, init_covariance,
				transition_matrix, observation_matrix,
				process_noise_covariance, measurement_noise_covariance);
```
</details>
<br>
<br>
<br>

![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/019a912f-8248-4744-a2c6-7ce9951eb3cc)
![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/a7a1838c-c44b-4e67-aeb1-7b1f40c7b5ca)
