
## Kalman Filter
In this project, a demonstration of the Kalman filter is presented to estimate the position of a vehicle with LIDAR & RADAR sensors. This demonstration is based on a simulator developed by Udacity and was completed as part of Udacity's Self-Driving Car Nanodegree. The [Udacity github repo](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) for this project has all the details about the software used in the simulator and the installation instructions.

### Summary of Setup Instructions
1. The project uses [uWebSocketIO](https://github.com/uNetworking/uWebSockets) for communication between the user-written algorithm and the simulator. Udacity has provided bash scripts to install this in both [Linux](https://github.com/chandrusuresh/CarND-Extended-Kalman-Filter-Project-master/blob/master/install-ubuntu.sh)/[Mac](https://github.com/chandrusuresh/CarND-Extended-Kalman-Filter-Project-master/blob/master/install-mac.sh) environments. These scripts are included in this repo.
2. The simulator can be downloaded from [here](https://github.com/udacity/self-driving-car-sim/releases).

### Basic Build Instructions (Linux/Mac)
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF`
This should setup a listener for the code to get data from the simulator.
5. Launch the simulator from a terminal: `./`
6. Select the Kalman Filter project and click start to start the simulation.

These steps should get any user in a Linux/Mac environment up and running with the code.

## Constant Velocity Model
In this project, the plant (car) dynamics is modeled by the constant velocity model. The state vector at time sample $k$ is represented as $x_k$ with 4 components: positions in the $x$ & $y$ coordinates $p_x$, $p_y$ and velocities in the $x$ & $y$ coordinates $v_x$,$v_y$.
The state transition model (plant dynamics) is defined by the following discrete time state space form.
$$ \begin{align*} \left[ \begin{matrix}  p_x\\ p_y \\ v_x\\v_y \end{matrix} \right]_{t=k+1} = \left[ \begin{matrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1\end{matrix}\right] \left[ \begin{matrix} p_x\\ p_y \\ v_x\\v_y \end{matrix} \right]_{t=k} + w_k\\ \end{align*}$$
where $w \sim \mathcal{N}(0,Q)$ is the $\textit{i.i.d}$  process noise from accelerations $\left[ \begin{matrix} a_x \\ a_y \end{matrix} \right]$. $\Delta t$ is the sampling time elapsed between time samples $k$ and $k+1$. Note that while the plant is modeled by the constant velocity model, the acceleration is modeled as noise and the complete model is suitable for tracking on highways where speeds are approximately a constant for the majority of the time. The process noise model is described in a separate section below.
The above plant model is expressed concisely as:
$$ \begin{equation} x_{k+1} = A x_k + w_k \tag{1} \end{equation}$$

## Sensor Model
In this project, a LIDAR and a RADAR sensors are modeled at different sample rates. The measurement model for these two sensors is described below.

### LIDAR
LIDAR (stands for LIght Detection And Ranging) is a time of flight sensor used for measuring distances between nearby objects by illuminating them with laser light and measuring the time taken for the reflected signal to be received by a sensor. A LIDAR sensor therefore measures the distance to a given object in 3D space. In this project, we work with 2D data and are only interested in the $x$ & $y$ coordinate frames.

The LIDAR sensor measurement model is defined as follows: The measurement vector $z_l$ is the vector of distances in the $x$ & $y$ coordinate frames represented by $z_{lx}$ & $z_{ly}$.
$$ \left[ \begin{matrix}  z_{lx} \\ z_{ly} \end{matrix} \right]_{t=k} = \left[ \begin{matrix} 1&0&0&0 \\ 0&1&0&0 \end{matrix} \right] \left[ \begin{matrix}  p_x\\ p_y \\v_x \\ v_y \end{matrix} \right]_{t=k} + v_{l,k} $$
where $v_l \sim \mathcal{N}(0,R_l) $ is $\textit{i.i.d}$ measurement noise. $R_l$ is the measurement covariance and in this project $R_l = \left[ \begin{matrix} 0 & 0 \\ 0 & 0\end{matrix} \right]$.
The concise expression for the LIDAR measurement model is: $$\begin{equation} z_{l,k} = H_l x_k + v_{l,k} \tag{2} \end{equation}$$

### RADAR
RADAR (stands for RAdio Detection And Ranging) is an object detection sensor that transmits radio waves in all directions and receives and processes the reflected waves to determine to determine the location of nearby objects. A RADAR sensor measures 3 quantities - the range $r$ of an object from the sensor, the bearing $\psi$ of the object from the sensor and the relative velocity $\dot{r}$ at which the object is moving w.r.t the sensor.

The RADAR sensor measurement model is defined as follows: The measurement vector $z_{r} = \left[ \begin{matrix} r & \psi & \dot{r} \end{matrix} \right]^T$.

$$ \left[ \begin{matrix}  r \\ \theta \\ \dot{r} \end{matrix} \right]_{t=k} = \left[ \begin{matrix} \sqrt{p_x^2 + p_y^2} \\ \tan^{-1}\left(\frac{p_y}{p_x}\right) \\ \frac{p_x v_x + p_y v_y}{\sqrt{p_x^2 + p_y^2}}\end{matrix} \right]_{t=k} + v_{r,k}$$
where $v_r \sim \mathcal{N}(0,R_r) $ is $\textit{i.i.d}$ measurement noise. $R_r$ is the measurement covariance and in this project $R_r = \left[ \begin{matrix} 0 & 0 \\ 0 & 0\end{matrix} \right]$.
The concise expression for the RADAR measurement model is: $$\begin{equation} z_{r,k} = F(x_k) + v_{r,k} \tag{3} \end{equation}$$ where $F(\cdot)$ is the nonlinear matrix expression above for the RADAR sensor model.



## Kalman Filter
The Kalman filter (also known as Linear Quadratic Estimator - LQE) is an algorithm that uses a series of sensor measurements to update the predictions of the states (unobserved/latent variables) from a known linear model of a plant. The algorithm takes into account noise in the sensor measurement as well as disturbanbes/inaccuracies in the plant model (in relation to the actual physical plant) to estimate the joint probability distribution over the unobserved variables at each sample time. The Kalman filter enables predictions to be updated when sensor measurements are available even if the measurements are made at irregular/infrequent intervals of time. The Kalman filter is also very extensible for fusing measurements from any number of sensors into the model for state estimation.

In this project, we use the LIDAR and RADAR as sensors to update the position and velocity of a vehicle. The Kalman filter is only suitable for linear models and therefore it is used to update the plant model predictions from LIDAR sensor measurements. For the RADAR, we use the Extended Kalman Filter (EKF) that is suitable for non-linear plant/sensor models. The EKF along with the extensions to the RADAR sensor model is detailed in the next section.

Assuming the plant dynamics from (1) and the LIDAR measurement model from (2), the prediction and measurement models are as follows:
$$ \begin{align*} x_{k} &= A x_{k-1} + w_{k} \\ 
z_{l,k} &= H x_k + v_{l,k}\end{align*}$$
The initial condition $x_0$ is assumed to be made after the "$0^\text{th}$" measurement was made. i.e., $x_0 = \mathbb{E}(x_{0|0})$.
### Prediction
1. Predicted State Estimate: $x_{k|k-1} = A x_{k-1|k-1}$
2. Predicted Covariance Estimate: $P_{k|k-1} = A P_{k-1|k-1} A^T + Q_k $

### Update
1. Measurement Residual: $y_k = z_{l,k} - H_l x_{k|k-1}$
2. Innovation Covariance: $S_k = H_l P_{k|k-1} H_l^T + R_{l,k}$
3. Optimal Kalman Gain: $K_k = P_{k|k-1} H_l^TS_l^{-1}$
4. Updated State Estimate: $x_{k|k} = x_{k|k-1} + K_k y_k$
5. Covariance Update: $P_{k|k} = (I-K_k H_l) P_{k|k-1}$


## Extended Kalman Filter (EKF)
The EKF is the nonlinear version of the Kalman Filter where the nonlinear equations whether in the sensor/plant model are linearized about the current mean and covariance. The linearized model is then used in the above kalman filter equations as described above.

The nonlinearities are linearized at each time step and therefore the prerequisite for this algorithm is that all the nonlinear funtions are differentiable and the Jacobian function can be computed.

Here we demonstrate the use of the EKF with a nonlinearity in the measurement model with the RADAR sensor. A nonlinearity in the plant model can be handled in a similar way.

### Linearizing RADAR measurement model

Linearizing the radar equation about the current state,
$$\begin{align*} \left[ \begin{matrix}  r \\ \theta \\ \dot{r} \end{matrix} \right]_{t=k} &= \left[ \begin{matrix} \sqrt{p_x^2 + p_y^2} \\ \tan^{-1}\left(\frac{p_y}{p_x}\right) \\ \frac{p_x v_x + p_y v_y}{\sqrt{p_x^2 + p_y^2}}\end{matrix} \right]_{t=k}  + v_{r,k} \\ & \approx \nabla_{x_{k-1|k-1}} \left( \left[ \begin{matrix} \sqrt{p_x^2 + p_y^2} \\ \tan^{-1}\left(\frac{p_y}{p_x}\right) \\ \frac{p_x v_x + p_y v_y}{\sqrt{p_x^2 + p_y^2}}\end{matrix} \right] \right) + v_{r,k} \\
&= \left[ \begin{matrix}  \frac{p_x v_x}{\sqrt{p_x^2 + p_y^2}} & \frac{p_y v_y}{\sqrt{p_x^2 + p_y^2}} & 0 & 0 \\ \frac{-p_y}{\sqrt{p_x^2 + p_y^2}}&\frac{p_x}{\sqrt{p_x^2 + p_y^2}}&0&0\\\frac{p_y (p_y v_x - p_x v_y)}{\sqrt{(p_x^2 + p_y^2)^3}}&\frac{p_x (p_x v_y - p_y v_x)}{\sqrt{(p_x^2 + p_y^2)^3}}&\frac{p_x}{\sqrt{p_x^2 + p_y^2}}&\frac{p_y}{\sqrt{p_x^2 + p_y^2}}\end{matrix} \right] x_{k-1} + v_r \\
\Rightarrow z_{r,k} &\approx H_{r,k-1} x_{k-1|k-1} + v_{r,k} \end{align*}$$

### EKF Procedure
### Prediction
1. Jacobian at current estimate: $H_{r,k-1} = \nabla_xF(x_{k-1|k-1})$
2. Predicted State Estimate: $x_{k|k-1} = A x_{k-1|k-1}$
3. Predicted Covariance Estimate: $P_{k|k-1} = A P_{k-1|k-1} A^T + Q_k $

### Update
1. Measurement Residual: $y_k = z_{r,k} - H_{r,k-1} x_{k|k-1}$
2. Innovation Covariance: $S_k = H_{r,k-1} P_{k|k-1} H_{r,k-1}^T + R_{r,k}$
3. Optimal Kalman Gain: $K_k = P_{k|k-1} H_{r,k-1}^TS_k^{-1}$
4. Updated State Estimate: $x_{k|k} = x_{k|k-1} + K_k y_k$
5. Covariance Update: $P_{k|k} = (I-K_k H_{r,k-1}) P_{k|k-1}$

## Process Noise Model
The plant is modeled by a simple constant velocity dynamics with acceleration as an $\textit{i.i.d}$ Gaussian noise. The acceleration in both $x$ & $y$ axes is an independent zero mean Gaussian random variable with standard deviation $\sigma = 3 \text{m/s^2}$. i.e., $a_x \sim \mathcal{N}(0,9)$ and $a_y \sim \mathcal{N}(0,9)$

The process covariance can be derived as:
$$ Q = \left[ \begin{matrix} \frac{\sigma_x^2 \Delta t^4}{4}&0&\frac{\sigma_x^2 \Delta t^3}{2}&0 \\ 0 & \frac{\sigma_y^2 \Delta t^4}{4}&0&\frac{\sigma_y^2 \Delta t^3}{2} \\ \frac{\sigma_x^2 \Delta t^3}{2}&0&\sigma_x^2 \Delta t^2 & 0 \\ 0 & \frac{\sigma_y^2 \Delta t^3}{2}&0&\sigma_y^2 \Delta t^2 \end{matrix} \right]$$


```python

```
