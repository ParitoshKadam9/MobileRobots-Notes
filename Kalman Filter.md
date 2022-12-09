3# Kalman Filter Localisation for mobile Robots:

#### # <u>Prediction Update and measurement Update</u> :

- **Prediction Update** : The robots position at time step t is predicted based on its old location (t-1) and its movement due to control input ${u(t)}$ (odometric estimation). 
- During ***prediction update***, the robot has no idea of where it is so it predicts its default location on the basis of previous data and predicts it ${t^{th}}$ location. 
- **Measurement Update** : 
	- Observation :  Sensor measurements, ${Z_t}$
	- Measurement prediction ${Z_t^0}$  : 
		- Predict observations are what the robots ***expects*** to see at present location ${x_t}$
		- Prediction expressed in sensor frame ${Z_t^\wedge = h^j(x_t, M^j)}$ 
	- **Matching** : 
		- Produce an assignment from the observations to the prediction.
		- ***innovation measure*** : measure of difference between predicted and observed feature.
		$$ v^{ij}_t = [Z_t^i-Z_t^\wedge] = [Z_t^i - h^j(x_t, m_j)]$$
			Here ${Z_t^i}$ is the observed position and ${Z_t^\wedge}$ is the measured state.
			
			![[Pasted image 20220915172200.png]]

		- ***Innovation covariance*** :
										$$ \sum = H^jP_t^\wedge G^{jT} + R_t^i$$
			- Here ${R_t^i}$ is the uncertainity of the sensor 
			- ${P_t}$ is the covariance at the measured position
			- ${H}$ is the jacobian 

		$$ x_t = x_t^\wedge + K_t.V_t$$
			- ${x_t}$ is the new position
			- Here ${x_t^o}$ is the predicted position
			- ${K_t}$ is the Kalman gain
			- ${V_t}$ is the innovation
		$$ K_t =P_t^\wedge H_t^T(\sum)^{-1} $$
		-  ***Covariance*** of the new position is given by 
			$$ P_t^\wedge = F_xP_{t-1}F_x^T + F_uQ_tF_u^T$$
			$$ P_t = P_t^\wedge - K_t(\sum)k_t^T$$
 			Here, ${Q_t}$ is the *sensor covariance*
			$$ Q_t = \begin{bmatrix} 
			k_r(\Delta s_r) & 0 \\ 0 & k_l(\Delta s_l) \end{bmatrix}$$ 
> The best estimate ${x_t}$ of the robot state at t is equal to the best prediction of the state at ${(t-1)}$ before the new measurement, ${Z_t}$, plus a correlation term of an optimal weighing value ${K_t}$ times the difference betweem ${Z_t}$ and best prediction ${Z_t^o}$ at time t

### Example : 

![[Pasted image 20220915183451.png]]

- After getting a ***control*** input ${u_t}$ , it moves to ${x_t^\wedge}$ 

### 1) Prediction updates :

- ${\hat x_t }$ is measured using *odometry* .
	$$ \hat x_t = f(x_{t-1}, u_t) $$
				$$ \hat P_t = F_xP_{t-1}F_x^T + F_uQ_tF_u^T$$
Here, ${Q_t}$ is the *sensor covariance*
			$$ Q_t = \begin{bmatrix} 
			k_r(\Delta s_r) & 0 \\ 0 & k_l(\Delta s_l) \end{bmatrix}$$
After using the relations for the differential robot , 

![[Pasted image 20220915185002.png]]

${\hat p}$ is basically ${\hat X_t}$ and M is the error 

### 2) Perception Updates :
	1) Observation (sensors)
	2) Measurement Prediction (maps)
	3) Matching
	4) Estimation

- Assume that the robot is using its sensors to collect all the info from its surroundings and model it's state.
- Basically ${Z_t}$ is what the robot will be sensing after coming to ${x_t}$
- Now, since the robot already has the map, the robot will check with the map from the position it is in, what it was ***supposed*** to see (${Z_t^\wedge}$)
- It can represent ***raw data scans*** as well as *features* like *lines, doors* or *any kind of landmarks*. 

![[Pasted image 20220915190617.png]]

$$ z_j^i = \begin{bmatrix} \alpha_t^i \\ r_t^i \end{bmatrix} $$
here, ${\alpha_t^i}$ is the angle and ${r_t^i}$ the radial distance.

##### Measurement Predicitions :

- We use the predicted robot position ${\hat x_t}$ and the map M to generate multiple predicted observations (${\hat Z_t}$)
$$ \hat Z_t = h^j(\hat x_t, M^j)$$
here, 

![[Pasted image 20220915191418.png]]

- The generated measurement predictions have to be transformed to the robot rame {R}
- The transformation is goven by 
$$ \hat Z_t = \begin{bmatrix} \hat \alpha_t \\ \hat r_t \end{bmatrix} = h^j(\hat x_t, m^j) = \begin{bmatrix}
\alpha_t-\hat \theta_t \\ r_t^j - \hat x_t.cos(\alpha_t) + \hat y_t.sin(\alpha_t) \end{bmatrix}$$

- ${r_t}$ is the distance of the feature from the ref frame/ world frame

Jacobian, 
$$ H_t^j = \begin{bmatrix} \delta\alpha\over\delta x^\wedge & \delta\alpha\over\delta y^\wedge &\delta\alpha\over\delta \theta^\wedge \\
\delta r \over\delta x^\wedge & ... & ...\end{bmatrix}  = \begin{bmatrix} 0 & 0 & -1 \\ cos(\alpha_t) & sin(\alpha_t) & 0\end{bmatrix} $$


##### Matching :
- For each measurement prediction for which corresponding observations is found we calculate the ***innovation*** :
	$$ v_t^{ij} = \begin{bmatrix} Z_t^i - Z_t^\wedge \end{bmatrix} = \begin{bmatrix} Z_t^i - h_j(x_t, m_j)\end{bmatrix}$$
	$$ = \begin{bmatrix} \alpha_t \\ r_t \end{bmatrix} - \begin{bmatrix}
\alpha_t-\theta_t^\wedge \\ r_t^j - x_t^\wedge.cos(\alpha_t) + y_t^\wedge.sin(\alpha_t) \end{bmatrix}$$
and the ***innovation covariance*** (error propagation law) : 

$$ \sum = H^j.P_t^\wedge.H^{jT} + R_t^i$$
here, ${R_t^i}$ is the ***sensor covariance***. 


![[Pasted image 20220915193757.png]]

- There is a difference bw what the robot is **seeing** and what the robot is **supposed** to see
- We will take this information to correct the position of the robot. 


##### Estimation (Applying the Kalman Filter) :

- Kalman filter gain : 
	$$ K_t =P_t^\wedge H_t^T(\sum)^{-1} $$
- Update of robot's position estimate : 
	$$ x_t = x_t^\wedge + K_t.V_t$$
- The associated variance :
	$$ P_t = P_t^\wedge - K_t(\sum)K_t^T$$
	
![[Pasted image 20220915194605.png]]




	   

