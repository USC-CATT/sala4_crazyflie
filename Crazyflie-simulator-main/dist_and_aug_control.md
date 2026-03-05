wind as a function of time

WIND_FORCE_AMPLITUDE * sin(2π f (t - t_start) + phase)

the max||position_error|| tracking start at 1 second after the wind starts. 


#### Augmented controll
 use the cascaded PID output as the baseline term, then add a virtual input \(v\).

1. Create a separate file `augmented_controller.py`
- Implement an `AugmentedController` wrapper class.
- It will call the existing PID controller internally (baseline untouched).

1. Add a mode switch in `runsim.py`
- Example: `controller_mode = "baseline"` or `"augmented"`.
- If `"augmented"`, instantiate/use `AugmentedController`; otherwise use current `controller.run(...)`.

1. Where to inject \(v\) in this simulator
- Best practical place: translational acceleration command path (x/y first).
- So augmented controller computes `v_xy` and adds it to desired accel before PID:
  - `des_state_aug.acc = des_state.acc + [v_x, v_y, 0]`
- Then pass `des_state_aug` into the existing cascaded PID.

1. Adapt your LS/filter equations per lateral axis
- Use 2-state channel \(x_i = [p_i,\dot p_i]\), \(A=\begin{bmatrix}0&1\\0&0\end{bmatrix}\), \(B=\begin{bmatrix}0\\1\end{bmatrix}\).
- Maintain digital twin \(\hat x_i\), error \(e_i=x_i-\hat x_i\).
- Estimate \(\dot e_i\) numerically, then filter:
  - `e_f_dot = (e_dot_est - e_f)/mu`
- Solve instantaneous LS:
  - `v_i = -(B^T B + eps)^(-1) B^T e_f_i`
- Add saturation on `v_i` (critical).

1. Start with XY only
- Since disturbance is lateral wind, do XY first.
- Keep Z and yaw unaugmented initially to reduce coupling/debug complexity.

1. Add debug signals
- Log/show `v_x`, `v_y`, `|e|`, `|e_f|` so we can verify cancellation is helping.

Important caveat:
- This won’t be mathematically identical to your paper’s single-input linear plant, because this simulator is nonlinear, cascaded, and underactuated. But this is the cleanest, least invasive practical equivalent.
