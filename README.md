# CSC 413 Robotics — Assignment 1: Motor Parameter Identification

**Student ID:** `240078`  **Robot:** TRIK, drive motors M4 (left) and M3 (right)

| Section | Content |
|---|---|
| 1 | Motor model: parameters, derivation of θ(t), assumptions |
| 2 | Test signals: choice and design check |
| 3 | Data-collection programs and measurement protocol |
| 4 | Data processing: step fits, ω<sub>ss</sub>(PWM), T<sub>m</sub>(PWM), k<sub>e</sub>, model vs. real motor |
| 5 | Artifact analysis |
| 6 | Report: parameter tables and analysis |

## 1. Motor model

$$
\dot{\omega} = \frac{U}{T_m k_e} - \frac{\omega}{T_m},
\qquad
\dot{\theta} = \omega
$$

The first equation says how fast the speed changes. The second says that the angle changes with the speed.

### 1.1 What the symbols mean

| Symbol | Meaning in simple words | Unit |
|---|---|---|
| $U$ | The voltage that drives the motor. TRIK does not set a voltage directly: it switches the full battery voltage on and off very quickly (PWM), and the motor feels the average. So $U = \frac{\mathrm{PWM}}{100}\,U_{battery}$: at 50 % PWM the motor gets about half the battery voltage. | V |
| $\theta$ | The angle of the wheel shaft. The encoder counts it in degrees; we convert it to radians. | rad |
| $\omega$ | How fast the shaft turns, angular velocity. $\omega = d\theta/dt$. | rad/s |
| $\omega_{ss}$ | The **final speed**: the speed the wheel settles at if the same voltage is kept on long enough. On a $\theta(t)$ plot it is the slope of the straight part. | rad/s |
| $T_m$ | The **time constant**: how quickly the motor gets up to speed, time that was spent to gain a wanted speed. After $T_m$ the motor has reached 63 % of its final speed; after $3T_m$ 95 %; after $5T_m$ 99 %. A heavier wheel or a weaker motor gives a larger $T_m$. | s |
| $k_e$ | The **back-EMF constant**: how many volts are needed for each rad/s of speed, volts spent for gaining an intended speed. A spinning motor also works as a generator and pushes back against the supply voltage. At the final speed this "push back" balances the supply, so $\omega_{ss} = U/k_e$. Because we measure at the wheel (after the gearbox), our $k_e$ also includes the gear ratio and some friction. | V·s/rad |

**Where the model comes from.** The motor has an electrical side, where the voltage pushes current through the
coil, and a mechanical side, where the current makes torque that speeds up the wheel. The faster the motor turns, the more voltage it
"pushes back" ($k_e\omega$), so the current and the torque get smaller. In the end the motor stops speeding up: that is the final
speed. If we ignore the coil's inductance and friction, the two sides combine into equation 5, with $T_m = \frac{JR}{k_m k_e}$
($J$ = inertia, $R$ = coil resistance, $k_m$ = torque constant).

### 1.2 How θ(t) follows from the model for a step

We switch on a constant voltage $U$ at $t = 0$. The motor starts at rest: $\omega(0) = 0$ and $\theta(0) = 0$.

**Step 1: write the equation in a nicer form.** Multiply equation 5 by $T_m$:

$$T_m\dot\omega + \omega = \frac{U}{k_e} = \omega_{ss}.$$

**Step 2: find the speed.** The solution is "final value + a part that dies out":
- if the speed is constant, $\dot\omega = 0$, so the constant solution is $\omega = \omega_{ss}$;
- the part that dies out solves $T_m\dot\omega + \omega = 0$, which gives $C e^{-t/T_m}$.

So $\omega(t) = \omega_{ss} + C e^{-t/T_m}$. At $t = 0$ the speed is 0, so $C = -\omega_{ss}$:

$$\omega(t) = \omega_{ss}\left(1 - e^{-t/T_m}\right).$$

The speed starts at 0 and rises smoothly towards $\omega_{ss}$.

**Step 3: find the angle.** The angle is the integral of the speed (with $\theta(0) = 0$):

$$\theta(t) = \int_0^t \omega_{ss}\left(1 - e^{-s/T_m}\right)ds
= \omega_{ss}\Big[s + T_m e^{-s/T_m}\Big]_0^t
= \omega_{ss}\left(t - T_m + T_m e^{-t/T_m}\right).$$

**Step 4: check the result.**
- At $t = 0$: $\theta(0) = \omega_{ss}(0 - T_m + T_m) = 0$. Correct.
- Taking the derivative gives back $\omega(t) = \omega_{ss}(1 - e^{-t/T_m})$, and at $t = 0$ it is 0, so the motor starts from rest.
  Correct.
- At $t = T_m$: $\omega = \omega_{ss}(1 - e^{-1}) = 0.632\,\omega_{ss}$. This is the "63 %" meaning of $T_m$.

**What this means.** After a while ($t \gg T_m$) the exponential part disappears and $\theta(t) \approx \omega_{ss}(t - T_m)$.
That is a straight line with slope $\omega_{ss}$, shifted to the right by $T_m$. So **one step gives both numbers**: the slope gives
$\omega_{ss}$, and the shift gives $T_m$. This is what `curve_fit` finds in section 4. Doing the step at many voltages also gives
$k_e$, from how $\omega_{ss}$ grows with $U$.

### 1.3 What the model assumes, and how we check it

| Assumption | Why it should be OK | How the data check it (what would show it is wrong) |
|---|---|---|
| **No inductance** ($L = 0$) | For a small motor the coil reacts in about 1 ms, much faster than $T_m \approx 0.1$ s and faster than one sample. | Look at the start of each step and at the chirp delay. Inductance would add a small extra delay at the start and more lag at high frequency than $\arctan(2\pi f T_m)$. |
| **Friction proportional to speed** (no "sticky" friction) | This keeps the model simple: double the voltage, double the speed, and zero voltage gives zero speed. | The $\omega_{ss}$ vs PWM plot. Sticky (static) friction shows up as a **dead zone**: at low PWM the wheel does not move, and the line does not go through zero. $T_m$ would also change with PWM. |
| **Constant load** | The robot is lifted, so each motor only turns its own wheel and gearbox. | $T_m$ and $k_e$ should be the same at every PWM and in both directions. Optional: one step with the robot on the floor (heavier load → larger $T_m$). |
| **Constant battery voltage during a run** | One run takes only about 1 minute. | Battery voltage before and after each series, and the repeated run: a weaker battery would give lower speeds. |

The model also quietly assumes that:
- PWM is proportional to voltage all the way up to 100 % (checked with **saturation**);
- the motor is the same in both directions (checked with **forward/backward asymmetry**);
- the encoder and the clock are perfect (checked with **quantisation**, **sampling jitter** and **zero drift**).

## 2. Test signals

One step shows the motor at only one speed. To learn how it behaves at many speeds, in both directions, and when the command changes
quickly, I used three kinds of input:

1. **Step series.** PWM from −100 % to +100 % in 10 % steps. Each step is held for 2 s, then the motor is switched off for
   1 s (still recording), and there are 3 s with the motor off at the very start. 2 s is much longer than $T_m$, so the wheel surely
   reaches its final speed; the first 0.3–0.5 s show how fast it gets there. This series also shows the dead zone, saturation, and
   the difference between forward and backward.
2. **Square wave** $A = 60\,\%$, $P = 1$ s, 8 s long (`sig = 1` in `motor_signal.qrs`). Why:
   * every half period (0.5 s) is a full step from full speed backward to full speed forward, or the reverse. So one run shows how fast
     the motor reacts **in both directions**, with the same battery;
   * it changes direction 16 times. Each time, the gears have to take up their play, so **gear backlash** would show up as a short
     delay or flat spot in the angle;
   * it jumps across zero instantly, so the wheel is near zero speed only briefly during each reversal. This makes sticking and
     backlash easy to separate from the rest of the motion.
3. **Chirp** $A = 60\,\%$, frequency rising from 0.2 to 5 Hz over $T = 10$ s
   (`sig = 4`, `T = 10000`). Why:
   * it shows how the motor follows faster and faster back-and-forth commands. The faster the command, the smaller the wheel's
     response should be ($\frac{1}{k_e\sqrt{1 + (2\pi f T_m)^2}}$) and the more it should lag behind ($\arctan(2\pi f T_m)$). This is a
     second, independent way to find $T_m$;
   * the frequency where the response starts to drop, $f_c = 1/(2\pi T_m)$, is about 0.8–3.2 Hz for $T_m$ = 0.05–0.2 s, which lies
     inside the sweep;
   * I stopped at 5 Hz on purpose. At higher frequencies the wheel would move only a few degrees per cycle (computed below), which is
     only a few encoder counts. The result would mostly show encoder steps and gear play, and with ~10 ms sampling there would be fewer
     than 20 samples per period.

**The formulas.** $\mathrm{frac}(x) = x - \lfloor x \rfloor$ is the part after the decimal point. TRIK's expression language has no
`round()`, so the program rounds with `floor(u + 0.5)`.

| Signal | `sig` | $u(t)$, % PWM |
|---|---|---|
| Square | 1 | $A$ if $\mathrm{frac}(t/P) < 0.5$, else $-A$ (written as $A(1 - 2\lfloor 2\,\mathrm{frac}(t/P)\rfloor)$) |
| Sawtooth | 2 | $A(2\,\mathrm{frac}(t/P) - 1)$ |
| Triangle | 3 | $A(1 - 4\,\lvert\mathrm{frac}(t/P) - 0.5\rvert)$ |
| Chirp | 4 | $A\sin\left(2\pi(f_0 t + (f_1 - f_0)t^2/(2T))\right)$, $0 \le t \le T$ |

**Checking the design rules**

| Rule | Square, A = 60 %, P = 1 s | Chirp, A = 60 %, 0.2–5 Hz, 10 s |
|---|---|---|
| Period several times $T_m$ | half period 0.5 s = 2.5–10 × $T_m$ ✓ | — |
| Chirp starts well below and ends well above $f_c$ | — | 0.2 Hz is 4–16 × below $f_c$; 5 Hz is 1.6–6 × above it (limited by the encoder, see above) |
| Crosses zero (both directions and the dead zone in one run) | ✓ | ✓ |
| Amplitude ≤ 80 % (not only full power) | 60 % ✓: well above the expected 5–15 % dead zone, and below full power | 60 % ✓ |

**After the measurements.** The measured $T_m$ is about 0.11–0.15 s (section 4.3). So each square-wave half period is 3–4.5 × $T_m$,
long enough for the wheel to get close to its final speed, and $f_c$ is about 1.1–1.4 Hz, inside the chirp range, as planned.

The cell below draws the commands the robot sends (same formulas, same rounding, ~10 ms sampling). The next cell checks the design
numbers for several possible $T_m$ values.

## 3. Data-collection programs and protocol

Both programs are TRIK Studio **visual** programs (no text code). Shown for the left motor (M4/E4); the right-motor copies
(`motor_steps_M3.qrs`, `motor_signal_M3.qrs`) differ only in motor port, encoder port/variable and file name.

**`motor_steps.qrs`**: Start → Remove File `steps_M4.txt` → Clear Encoder E4 → Expression `level = -100; step = 10; t0 = time()` →
loop *while* `time() - t0 < 3000` { Expression (log θ, t, 0) → Write To File → Timer 5 ms } →
loop *while* `level <= 100` { Condition `level != 0` → *true*: `t0 = time()` → Motor M4 = `level` →
loop 2 s { log θ, t, level → Write → Timer 5 ms } → Motor M4 = 0 → loop until 3 s { log θ, t, 0 → Write → Timer 5 ms };
both branches → `level = level + step` } → Stop Motors M4 → End.

**`motor_signal.qrs`**: Start → Remove File `signal_M4.txt` → Clear Encoder E4 → Expression with the parameters
`sig = 1; A = 60; P = 1000; T = 8000; f0 = 0.2; f1 = 5.0; t0 = time()` → loop *while* `time() - t0 < T`
{ `tau`, `ph` → chain of Conditions on `sig` selecting square / sawtooth / triangle / chirp → `u = floor(uf + 0.5)` →
Motor M4 = `u` → log θ, t, u → Write → Timer 5 ms } → Motor M4 = 0 → 1 s of logged coasting → Stop Motors → End.

Every line of every file is `angle_deg,time_ms,pwm`.

**Protocol**

1. Robot lifted on a stand so both wheels spin freely.
2. Battery voltage read from the TRIK display before and after each run and written into `battery.txt`.
3. `motor_steps.qrs` on M4 twice (repeatability), then on M3 (`motor_steps_M3.qrs`).
4. `motor_signal.qrs` on M4 and M3 with the square wave (`sig = 1`), then with the chirp (`sig = 4`, `T = 10000`).
5. The programs overwrite their file on every run, so each file is downloaded right after its run (content unchanged) and saved as:

| Run | Saved as |
|---|---|
| steps, left, 1st / 2nd run | `steps_M4_run1_078.txt`, `steps_M4_run2_078.txt` |
| steps, right | `steps_M3_078.txt` |
| square wave | `signal_M4_square_078.txt`, `signal_M3_square_078.txt` |
| chirp | `signal_M4_chirp_078.txt`, `signal_M3_chirp_078.txt` |

## 4. Data processing

### 4.0 Configuration and loading
Settings that must match what was actually run on the robot are in google colab notebook.

### 4.1 Segmentation and encoder direction
The step file is split by the PWM column: every run of identical non-zero PWM is one step, and the PWM = 0 rows that follow it are its
coasting part. The rows before the first step are the 3 s zero-drift record.
Depending on how a motor is mounted, its encoder may count negative for positive PWM. The sign is detected from the +100 % step and
applied to all files of that motor, so that "forward" always means positive PWM and positive speed.

### 4.2 Step responses
For every step, the initial angle and time are subtracted and the model from section 1.2 is fitted with `optimize.curve_fit`.
The standard errors come from the diagonal of the covariance matrix. Steps where the wheel turned less than `MIN_TURN_DEG` are marked
as *not moving* (dead zone): there the fit has no meaning.


### 4.3 Speed and time constant versus PWM

For each motor and direction a straight line $|\omega_{ss}| = a\,(|\mathrm{PWM}| - \mathrm{PWM}_{0})$ is fitted to the **linear part**.
The measured curve is not straight over the whole range: it is steep at low PWM and straight only above about 40 %
(Figure 4.2), so the linear part is set to **40–100 %** (`LINEAR_RANGE`; `{}` switches to the automatic search, which ends the
linear part where a point leaves the line by more than `SAT_TOL`). Levels above the linear part would be **saturation**.

For an ideal motor with dry friction, the zero crossing $\mathrm{PWM}_{0}$ of the line would be the **dead zone**. Here it is
*negative* (the line hits zero speed below 0 %), so it is an offset of the high-PWM line, not a dead zone. The dead zone is
taken from the data instead: the bracket between the largest level that did not move and the smallest one that did.
All ±10 % steps moved, so the dead zone is below 10 % and is not resolved by 10 % steps (section 5.3 narrows it).

$T_m$ of a direction is the **median** over the linear part, using only good fits (RMS residual ≤ `FIT_RMSE_MAX`).
The median is used because single steps (e.g. the first step after a direction change, which starts with the gear play on the
wrong side) give outliers.

Conversion to voltage: $U = \frac{\mathrm{PWM}}{100}U_{battery}$, so the slope in volts is $a\cdot 100/U_{battery}$ and
$k_e = \frac{U}{\omega_{ss}} = \frac{U_{battery}}{100\,a}$.

### 4.4 Complex signal: model versus real motor

The identified parameters are used to simulate the motor with the **recorded** input. The robot holds each PWM command until the next
cycle, so the input is applied as a zero-order hold (`np.searchsorted`), which is equivalent to `np.interp` but without smoothing the
steps of the square wave. Four model variants are compared:

| Variant | Parameters | Input nonlinearity |
|---|---|---|
| **A: linear** | constant $k_e$, $T_m$ from 4.3, one set per motor (mean of both directions) | none, $u = U$ |
| **B: dead-zone cut** | as A | $u = 0$ when $\lvert\mathrm{PWM}\rvert$ is below the middle of the dead-zone bracket, else $u = U$ (as suggested in the assignment) |
| **C: affine, per direction** | forward set for $U > 0$, backward set for $U < 0$ | $u = \mathrm{sign}(U)\max(\lvert U\rvert - U_0, 0)$ with the line of 4.3; because $\mathrm{PWM}_0 < 0$ this adds an offset rather than removing a dead zone |
| **D: gain-scheduled first order** | $\omega_{ss}(\mathrm{PWM})$ and $T_m(\mathrm{PWM})$ interpolated from the step table (median of the runs, both directions); at PWM = 0 the coasting time constant from 5.5 | $\dot\omega = (\omega_{ss}(\mathrm{PWM}) - \omega)/T_m(\mathrm{PWM})$ |
| **E: measured speed curve, constant $T_m$** | $\omega_{ss}(\mathrm{PWM})$ table as in D; $T_m$ = median of the linear part (mean of both directions) | $\dot\omega = (\omega_{ss}(\mathrm{PWM}) - \omega)/T_m$ |

A–C are the first-order model with constant $T_m$ and $k_e$ asked for in the assignment. D and E keep the first-order structure but use
the speed actually measured at each PWM level instead of one straight line; they use only numbers already measured in the step series.
E separates a static nonlinearity (how fast the wheel turns at each PWM) from linear first-order dynamics with one $T_m$; it is the
reference model in the figures below because it has the best speed fit on every record.

**How the comparison is made.** Before comparing speeds, the simulated angle goes through the same steps as the measured
one (`np.gradient`, then the moving average). That way the smoothing is not counted as a model error.

Two numbers are reported for each record:
- **Angle fit:** small speed errors add up over time, so the angle slowly drifts away from the model. This makes the angle fit low
  even when the model follows the motion well, especially for the chirp, where the wheel only swings 10–50° per cycle.
- **Speed fit:** this shows better how well the model follows the motor's motion.

#### Chirp: frequency response
For each full cycle of the chirp phase $\varphi(t)$, the raw speed is fitted by least squares with $a\sin\varphi + b\cos\varphi + c$.
This gives the speed amplitude $\sqrt{a^2 + b^2}$ and the phase lag at the instantaneous frequency
$f(t) = f_0 + (f_1 - f_0)t/T$. The amplitude ratio (speed amplitude / input amplitude in volts) is compared with
$\frac{1}{k_e\sqrt{1 + (2\pi f T_m)^2}}$ and the lag with $\arctan(2\pi f T_m)$.

**Chirp result.**
- **At slow frequencies (below 2 Hz) the simple model works.** The measured delay of the wheel matches the formula $\arctan(2\pi f T_m)$
  within about 5°. From this delay, $T_m$ comes out as 0.13–0.17 s, almost the same as from the steps (0.11–0.15 s). So the chirp
  confirms $T_m$ in a second, independent way.
- **The response gets smaller at higher frequency, as expected.** From 0.6 Hz to 4.5 Hz it drops about 4 times
  (from 1.2–1.3 to 0.30–0.34 rad/(s·V)).
- **The measured points lie above the formula line everywhere.** The reason is that $k_e$ comes from the 40–100 % part of the
  speed curve, and that line gives too low speeds at medium PWM (Figure 4.2).
- **At fast frequencies (above 2 Hz) the wheel is quicker than the model.** It moves about twice as much as model E predicts
  (0.30–0.34 vs 0.15–0.19 rad/(s·V) at 4–5 Hz) and lags less (72–78° vs 87–94°). When the motor is reversed quickly and hard, it
  reacts faster than one $T_m$ of about 0.15 s. This matches Figure 4.3, where $T_m$ is only 0.03–0.06 s at high PWM.

  ## 5. Artifact analysis
Each artifact from the assignment table is measured below. The interpretation is in section 6.

