## Autonomous Map Building

 - A mobile robot should be able to autonomously explore the environment with its onboard sensors, gain knowledge, interpret the scene, build an appropriate map and localizr itself relative to this map.
 
> SLAM = Simultaneous Localization and Mapping

##### Mathematical formulation of SLAM :

${X_t}$ ------------- Robot Path
${U_t}$ ------------- Robot motion
${M}$ ------------- True map
${Z_t}$ ------------- Observations in the sensor frame

- The SLAM problem is recovering the ***model map*** ${M}$ and ***robot path*** ${X}$ from ***odometry*** ${U}$ and ***observations*** ${Z}$

# Extended Kalman Filter SLAM

1) Extended state vector ${y_t = (x_t, m_0, m_1,...m_{n-1})^T}$ 
2) Prediction (same)
3) Measurement model (observation)
4) Estimation 