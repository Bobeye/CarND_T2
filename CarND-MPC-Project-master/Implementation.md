# Implementation

### The Model
The model has 6 states including x, y, v, psi, cte, epsi and 2 actuators including steering (delta, in radians) and throttle (accleration a).
s, y, v, psi is defined under vehicle coordinates, which makes x, y, psi all to 0, the v is from the message.
cte and epsi are caculated based on the 3rd order polynomial fitting with ptsx and ptsy from the message.
The update function from t to t+1 is from the in-class quiz:
```
// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
// v_[t+1] = v[t] + a[t] * dt
// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

### N and dt
I started from setting up a 5-second prediction range. I tried several differnt dt choices from 0.05s to 0.5s. The final program is setting at dt=0.2s, therefore N=25.
For further analysis, a bigger N may lead to longer computing time (too many points) and lower accuracy (too far away from time t that makes polinomial fitting less accurate), a small t is good for waypoints approximation but is bad for dealing with latency.

### Polynomial Fitting and MPC Preprocessing
The original waypoints, ptsx and ptsy and vehicle coordinates, x and y are on global coordinates. Transfering to vehicle coordinates improves the performance. The function for coordinates transfer is:
```
// vehilce_x[i] = (ptsx[i]-x)*cos(psi)+(ptsy[i]-y)*sin(psi)
// vehilce_y[i] = -(ptsx[i]-x)*sin(psi)+(ptsy[i]-y)*cos(psi)
```
A 3rd order polynomial is set to fit the track because of the predicted route is curved. The MPC process is:

setup>>

* Set N & dt
* Define vehicle dynamics, actuator & non-actuator limitations
* Define the cost function

loop>>

* pass the initial state to mpc (reset x, y ,psi to 0 due to vehicle coordinate transfer)
* call the solver
* apply the first actuator control to the vehicle

### Latency handling
The weight tuning on the cost function helps dealing with the latency. The 0.2s dt is longer than the 100ms latency which also helps improving the performance. The reference velociy is tuned to 40 that guarantees a more stable performance of the vehicle.

### Video demo
https://youtu.be/Q7sOFbKKwDs
