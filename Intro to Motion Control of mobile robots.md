## Kinematic control 

Forward differential kinematics : ${ \eta^o = J(\eta)\zeta}$ 
inverse differential kinematics = ${\zeta = J^{-1}(\eta)\eta^o}$ 

- basically the inverse differential is a feedback.

#### Robot Kinematoc (motion) control :
1) Desired :
	- Desired position , ${\eta_d(t)}$
	- Desired velocities, ${\eta_d^o d(t)}$, for set-point control : ${\eta_d^o(t)=0}$

2) Available :
	- Actual positions, ${\eta(t)}$
	- Jacobian matrix, ${J(\eta), J^{-1}(\eta)}$

3) To Find :
	- Control inputs ${\zeta}$

4) Objective :
	- Exponentially stable, t=>inf, ${\eta=> \eta_d}$
	- basically, t=> inf and ${\eta^d => 0}$


$$ \eta^d = \eta_d - \eta $$
here, ${t => \inf}$ and ${\eta^d => 0}$


![[Pasted image 20220916161701.png]]
 - similar to this

Therefore, 

![[Pasted image 20220916161817.png]]

- This feed forward (velocity based) control will not work when ${n_d(t=0)!=\eta(t=0) or \eta_d^o =0}$\
	- In this case, a feedback control (closed-loop control) will work.
	$$ \eta^d(t) = e^{-\lambda t}$$ here, ${\lambda}$ is a ***diagonal matrix***

	differenciating the equation w.r.t time 
	$$ \eta^{`d} (t) + \lambda \eta^d(t)=0$$
$$ \eta_d^`(t) - \eta^`(t) + \lambda \eta^{`d} (t) =0 $$
 - for set-point control, ${\eta_d^`(t)= 0}$

Therefore,
$$ \zeta = J^{-1}(\eta)[\lambda \eta^d(t)]$$
 - This is a simple closed-loop feedback control, we simply called it as proportional control.


# Dynamic Control

- Every desired information will be given 
- ![[Pasted image 20220919155531.png]]



