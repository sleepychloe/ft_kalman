Mandatory part + every Optional part

Tested on Linux

finished but not submitted yet

## Lists
 * [Demo](#demo) <br>
 * [Project ft_kalman](#project-ft-kalman) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Introduction](#project-ft-kalman-introduction) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Project Description](#project-ft-kalman-description) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Mandatory Part](#project-ft-kalman-mandatory-part) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Optional Part](#project-ft-kalman-optional-part) <br>
 * [Installation & Usage](#installation-usage) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Installation](#installation) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Usage](#usage) <br>
 * [Graphics](#graphics) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Sections](#graphics-sections) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Keyboard and Mouse Control](#graphics-control) <br>
 * [Kalman Filter](#kalman-filter) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Kalman Filter and Adaptive Kalman filter<K>](#kalman-filter-kalman-filter) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [kalman filter](#kalman-filter-kalman) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [adaptive kalman filter](#kalman-filter-adaptive-kalman) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Class Template KalmanFilter<K>](#kalman-filter-class-template) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [predict without control input](#kalman-filter-class-template-1) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [predict with control input](#kalman-filter-class-template-2) <br> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [kalman filter update](#kalman-filter-class-template-3) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [adaptive kalman filter update](#kalman-filter-class-template-4) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [How To Calculate](#kalman-filter-how-to-calculate) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [How To Initialize](#kalman-filter-how-to-initialize) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [x̂ₖ, Fₖ](#kalman-filter-how-to-initialize-1) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [B, uₖ](#kalman-filter-how-to-initialize-2) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [Pₖ](#kalman-filter-how-to-initialize-3) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [Qₖ](#kalman-filter-how-to-initialize-4) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;⋅ [zₖ, Hₖ, Rₖ & ỹₖ, Sₖ](#kalman-filter-how-to-initialize-5) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [Initial Values](#kalman-filter-initial-values) <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- [PDFs](#kalman-filter-pdfs) <br>

<br>

## Demo <a name="demo"></a>
![Animated GIF](https://github.com/sleepychloe/ft_kalman/blob/main/img/gif00_graph.gif)
###### ↳ trajectory duration: 42, seed: 42, filter speed: no, graph: yes, noise: default, adaptive kalman filter: no

<br>

### seed

<img src="https://github.com/sleepychloe/ft_kalman/assets/78352910/faa4414a-8d07-4ab8-9756-bfe0475f8392" width="600" height="400">

###### ↳ trajectory duration: 90, seed: 42, filter speed: no, graph: yes, noise: default, adaptive kalman filter: no

<br>

<img src="https://github.com/sleepychloe/ft_kalman/assets/78352910/4bf70ff8-3863-4ea8-83d5-2161892b9dae" width="600" height="400">

###### ↳ trajectory duration: 90, seed: entropy, filter speed: no, graph: yes, noise: default, adaptive kalman filter: no

<br>

<img src="https://github.com/sleepychloe/ft_kalman/assets/78352910/7696646e-e81e-43cd-8793-d7565599d994" width="600" height="400">

###### ↳ trajectory duration: 90, seed: entropy, filter speed: no, graph: yes, noise: default, adaptive kalman filter: no

<br>
<br>

### filter speed

<img src="https://github.com/sleepychloe/ft_kalman/assets/78352910/d7fd4bde-b6cd-4627-b351-990f3f0ff86f" width="600" height="400">

###### ↳ trajectory duration: 5, seed: 42, filter speed: yes, graph: no, noise: default, adaptive kalman filter: no

<br>
<br>

### noise

<img src="https://github.com/sleepychloe/ft_kalman/assets/78352910/1a41cf64-f977-42a7-9003-5dea84ea2890" width="600" height="400">

###### ↳ trajectory duration: 2, seed: 42, filter speed: no, graph: no, noise: 2, adaptive kalman filter: no

<br>
<br>

### adaptive kalman filter

<img src="https://github.com/sleepychloe/ft_kalman/assets/78352910/f80caeef-ca25-4700-8792-61aa7ffcd025" width="600" height="400">

###### ↳ trajectory duration: 90, seed: 42, filter speed: no, graph: no, noise: 0, adaptive kalman filter: no

<br>

<img src="https://github.com/sleepychloe/ft_kalman/assets/78352910/de4201cd-d545-4068-b381-f0f8af165925" width="600" height="400">

###### ↳ trajectory duration: 90, seed: 42, filter speed: no, graph: no, noise: 0, adaptive kalman filter: yes

<br>
<br>

## Project ft_kalman <a name="project-ft-kalman"></a>

### Introduction <a name="project-ft-kalman-introduction"></a>

This project is focused on implementing a Kalman Filter to accurately estimate the position of a vehicle based on noisy sensor data.<br>
<br>
The Kalman Filter is a powerful algorithm used in various fields such as navigation, economics, and computer vision to process and estimate the state of a dynamic system from a series of incomplete and noisy measurements.<br>
<br>

### Project Description <a name="project-ft-kalman-description"></a>

This project simulates the Inertial Measurement Unit (IMU) of a generic vehicle moving in a simplified environment.<br>
The vehicle moves along its longitudinal axis without the influence of air resistance or gravity.<br>
<br>
The following inputs are provided:<br>
 ⋅ True initial position (X, Y, Z in meters, at the beginning)<br>
 ⋅ True initial speed (in km/h, at the beginning)<br>
 ⋅ Current acceleration (in m/s², every 0.01 seconds)<br>
 ⋅ Current direction (in euler angles)<br>
 ⋅ Current GPS position (X, Y, Z in meters, every 3 seconds)<br>
<br>
These measurements are affected by Gaussian white noise with the following characteristics:<br>
 ⋅ Accelerometer: σ = 10⁻³, v = 0<br>
 ⋅ Gyroscope: σ = 10⁻², v = 0<br>
 ⋅ GPS: σ = 10⁻¹, v = 0<br>
<br>
The goal is to implement a Kalman Filter that processes these noisy measurements and provides accurate position estimates.<br>
<br>

### Mandatory Part <a name="project-ft-kalman-mandatory-part"></a>

The primary objective is to develop a robust Kalman Filter capable of handling real-time trajectory estimation for up to 90 minutes.<br>
<br>
The filter must be optimized to prevent timeouts and maintain an estimation accuracy within 5 meters of the true position.<br>
<br>

### Optional Part <a name="project-ft-kalman-optional-part"></a>

 ⋅ Develop a trajectory visualizer<br>
&nbsp;&nbsp;&nbsp;(2D plot or 3D visualizer with HUD and variance display)<br>
 ⋅ Optimize the filter for faster computation<br>
&nbsp;&nbsp;&nbsp;(mean computation time in milliseconds)<br>
 ⋅ Enhance the filter to handle higher noise levels than default noise amount<br>
 ⋅ Introduce additional innovative functionalities beyond the basic requirements<br>
&nbsp;&nbsp;&nbsp;: Implementing adaptive kalman filter<br>
<br>
<br>

## Installation & Usage <a name="installation-usage"></a>

### Installation <a name="installation"></a>

```
  git clone https://github.com/sleepychloe/ft_kalman.git
  cd ft_kalman
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

### Usage <a name="usage"></a>

Before running program, allow the root user on your local system to access the X server via
```
  xhost +local:root
```

To run program,
```
   make (d=[DURATION] e=[ENTROPY FLAG] s=[FILTER SPEED FLAG] g=[GRAPH FLAG] n=[NOISE INCREASE] adaptive=[ADAPTIVE FILTER FLAG])
```
 ⋅ default trajectory duration = 90<br>
 ⋅ default entropy flag = 0<br>
 ⋅ default filter speed flag = 0<br>
 ⋅ default graph flag = 1<br>
 ⋅ default noise increase = 1<br>
 ⋅ default adaptive filter flag = 0<br>

<details>
<summary><b><ins>examples</ins></b></summary>


 ⋅ ex1: trajectory duration: 90,<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;seed: 42,<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;printing filter speed: no,<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;showing it's graph: yes<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;noise increase: 1<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;compute with adaptive kalman filter: no<br>

```
   make
```

 ⋅ ex2: trajectory duration: 30,<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;seed: generated from entropy,<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;printing filter speed: yes,<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;showing it's graph: no<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;noise increase: 2<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;compute with adaptive kalman filter: yes<br>
```
   make d=30 e=1 s=1 g=0 n=2 adaptive=1
```

</details>
<br>

To see lists of containers, volumes, images, and networks,
```
   make list
```

To see outputs of containers,
```
   make logs
```

To stop containers,
```
   make stop
```

To restart containers,
```
   make restart
```

To clean every containers, volumes, images, and networks,
```
   make fclean
```
<br>

When you are done testing, do not forget to disable root access via
```
  xhost -local:root
```
<br>
<br>

## Graphics <a name="graphics"></a>

### Sections <a name="graphics-sections"></a>

<img src="https://github.com/sleepychloe/ft_kalman/assets/78352910/5c77aa95-e608-44a5-ba49-68092bbf946d" width="600" height="400">

<br>
<br>

### Keyboard and Mouse Control <a name="graphics-control"></a>

Every mouse and keyboard control depends on the cursor's position relative to the window's width and height.<br>
```
╔═════════════╦═══════════════════╗
║ SCROLL_DOWN ║ zoom out          ║
║ SCROLL_UP   ║ zoom in           ║
║ KEY_A       ║ camera vector -x  ║
║ KEY_D       ║ camera vector +x  ║
║ KEY_S       ║ camera vector -y  ║
║ KEY_W       ║ camera vector +y  ║
║ KEY_DOWN    ║ camera vector -z  ║
║ KEY_UP      ║ camera vector +z  ║
╚═════════════╩═══════════════════╝
```
<br>

![Animated GIF](https://github.com/sleepychloe/ft_kalman/blob/main/img/gif01_control.gif)
###### ↳ control position graph: cursor is on the position section(left half of the window)<br>
<br>
<br>

## Kalman filter <a name="kalman-filter"></a>
### Kalman Kilter and Adaptive Kalman Filter <a name="kalman-filter-kalman-filter"></a>
#### kalman filter <a name="kalman-filter-kalman"></a>

The Kalman Filter is a recursive algorithm used to estimate the state of a dynamic system from a series of noisy measurements.<br>
<br>
It operates in two steps: prediction and update.<br>
 ⋅ Prediction: The filter predicts the state of the system and its uncertainty using the process model.<br>
 ⋅ Update: The filter updates the predicted state and uncertainty with new measurements using the observation model.<br>
<br>
The Kalman Filter is widely used in navigation, control systems, and time-series analysis due to its efficiency and ability to provide accurate estimates in real-time.<br>
<br>

#### adaptive kalman filter <a name="kalman-filter-adaptive-kalman"></a>

The Adaptive Kalman Filter extends the standard Kalman Filter by dynamically adjusting the process and measurement noise covariances.<br>
This adaptation allows the filter to maintain optimal performance even when the noise characteristics change over time.<br>
<br>
The adaptive Kalman Filter is particularly useful in environments with varying noise levels, enhancing the robustness and accuracy of state estimation.<br>
<br>
<br>

### Class Template KalmanFilter<K> <a name="kalman-filter-class-template"></a>
#### predict without control input <a name="kalman-filter-class-template-1"></a>
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
<br>

#### predict with control input  <a name="kalman-filter-class-template-2"></a>
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
<br>

#### kalman filter update  <a name="kalman-filter-class-template-3"></a>
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

### adaptive kalman filter update <a name="kalman-filter-class-template-4">
 ⋅ adaptive measurement noise covariance Rₖ = (1 − α) * Rₖ + α * Sₖ<br>
 ⋅ adaptive process noise covariance Qₖ = (1 - α) * Qₖ + α * Pₖ<br>
&nbsp;&nbsp;(α: adaptation rate,<br>
&nbsp;&nbsp;&nbsp;R: measurement noise covariance matrix,<br>
&nbsp;&nbsp;&nbsp;Q: process noise covariance)<br>
```
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

template <typename K>
void	AdaptiveKalmanFilter<K>::adaptNoiseCovariance(Matrix<K> innovation_covariance)
{
	this->_measurement_noise_covariance = (1 - this->_adaptation_rate) * this->_measurement_noise_covariance
						+ this->_adaptation_rate * innovation_covariance;
	this->_process_noise_covariance = (1 - this->_adaptation_rate) * this->_process_noise_covariance
						+ this->_adaptation_rate * this->_covariance;
}
```
<br>

### How To Calculate <a name="kalman-filter-how-to-calculate"></a>

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

### How To Initialize <a name="kalman-filter-how-to-initialize"></a>
#### x̂ₖ, Fₖ <a name="kalman-filter-how-to-initialize-1"></a>

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

<br>

#### B, uₖ <a name="kalman-filter-how-to-initialize-2"></a>
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

#### Pₖ <a name="kalman-filter-how-to-initialize-3"></a>

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

#### Qₖ <a name="kalman-filter-how-to-initialize-4"></a>

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

#### zₖ, Hₖ, Rₖ & ỹₖ, Sₖ <a name="kalman-filter-how-to-initialize-5"></a>

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

### Initial Values <a name="kalman-filter-initial-values"></a>

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

### PDFs <a name="kalman-filter-pdfs"></a>

![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/eeb83879-2c88-4860-9456-7e993d5a0297)
![image](https://github.com/sleepychloe/ft_kalman/assets/78352910/026dddb7-574a-4ea7-a8bc-f4b667af7809)
