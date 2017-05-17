# Implementation

### The Model
The model has 6 states including x, y, v, psi, cte, epsi and 2 actuators including steering and throttle.
The update function is from the in-class quiz:
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

### Latency handling
The 0.2s dt is longer than the 100ms latency which helps improving the performance.
