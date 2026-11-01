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

**M4 is not the same in both directions; M3 almost is.**
- **M4** turns 6–8 % faster forward than backward at the same PWM (7.6 % at 10 %, 6.5 % at 100 %). Its forward slope is 10 % higher.
- **M3** differs by only 1 % at 10 % and 2.6 % at 100 % (slope difference 7 %).
- **$T_m$:** the difference between directions is not reliable, because $T_m$ changes much more with PWM than with direction
  (Figure 4.3).

The two runs of the same motor agree within about 1–2 % (section 5.8), so the 6–8 % difference of M4 is real. You can see it in the
square wave: M4's angle moves a bit further forward every period (Figure 4.4).

Possible reasons: the brushes inside the motor are not exactly centred, the gearbox has more friction in one direction, or the two
halves of the motor driver are not identical.

**The encoder does not drift.** When the motor was off for 3 s, the encoder did not change by even one count, in all three
records. Every M4 coasting window also ended with the wheel standing still.

Three other things were found:
1. **The clock does not start at 0.** The first `time()` value is 20–21 ms in the step files and 29–37 ms in the signal files. This is the
   time spent on Remove File, Clear Encoder and the first loop. The notebook subtracts the first time value of every segment.
2. **`steps_M3_078.txt` holds two runs, one after the other.** The time jumps back to the start at row 3045, because the file was not
   deleted between the two runs. The first run has no motor-off part; it starts directly with the −100 % step at 3050 ms. The
   notebook splits the file into two runs without changing it.
3. **The wheel does not end at the same angle after a series** (M4: +710° after run 1, +322° after run 2). This is not drift. The
   series is symmetric in PWM, but the wheel travels a bit more forward than backward and does not coast the same distance every
   time, and these differences add up. The M3 values in the "last 0.3 s of coasting" column (8–30 counts) mean that the wheel was still
   turning when the next step started (section 5.5), not that the encoder drifted.

**Dead zone: below 10 %. Starting from rest, about 9–15 %.**
- Every ±10 % step made both wheels turn, at 3.8–4.7 rad/s (about 30 % of full speed). So the dead zone is below 10 %, and steps of
  10 % are too coarse to measure it exactly.
- The chirp starts from rest and increases PWM slowly from 0 %. M4 first moved between 9 and 12 %, M3 between 15 and 18 %. These values
  are a little too high, because the motor reacts with some delay.

Together this means the wheel has to **break free first** (static friction). Once it is free, it jumps straight to a high speed: at
±10 % the wheel is almost at full speed from the very first sample, with no smooth start. That is why two of the five failed step fits
in section 4.2 are the ±10 % steps.

The straight line through the 40–100 % part reaches zero speed at −32…−62 %. So the line cannot be used to find the dead zone here:
at low PWM the real speeds are far above the line (Figure 4.2).

**Saturation: none.** At 100 % the speed is still on the straight line (101–102 % of it). The curve is bent only at low PWM, not at
the top.

**After power-off the wheel spins on freely.**
- **How far:** after the motor is set to 0, the wheel turns another 2.2–2.3 rad (M4) and 3.2–3.6 rad (M3), median over all steps.
  From full speed it is 170–390° (3–6.8 rad). With the 3.75 cm wheel radius from the course example, that would be 11–25 cm of
  travel.
- **How fast it slows down:** the "coasting time constant" (coasting angle divided by the speed before power-off) is 0.24 s for M4
  and 0.29–0.32 s for M3. That is about twice the normal $T_m$, and 5–10 times $T_m$ at high PWM.
- **Why:** in TRIK, power 0 simply switches the motor driver off; braking is a separate command. Nothing actively brakes the motor,
  so only friction slows the wheel down.

M3 has less friction than M4: after the −100 % step it was still turning 30° in the last 0.3 s of the 1 s coasting window, so the next
step started while the wheel was still moving a little. For the model this means that PWM = 0 needs its own, slower time constant
(model D uses it).

**Gear backlash is too small to measure here.**
- In the square wave, the measured speed changes sign 86–117 ms after the command reverses. That is the same time as model E
  (109–117 ms), or up to 22 ms earlier.
- After a reversal, the encoder stays at the same value for only 10–26 ms on average, about one sample.

So the play in the gears does not cause a delay we can see. One sample is about 27 ms and one encoder count is 1°, and the effect is
smaller than that.

Figure 5.3 also shows that after a reversal the real wheel turns around a little earlier, and goes 5–8° less far, than model E. When
full power is applied in the opposite direction, the motor reverses faster than one $T_m$ of about 0.15 s predicts, the same effect as
in the chirp.

Backlash can still be seen indirectly in two places:
- **In the step series:** the ±10 % steps start suddenly instead of smoothly (section 5.3).
- **In the chirp at 2–5 Hz:** here the wheel swings only 10–50° per cycle, and it does not come back to the same angle each cycle. It
  shifts by up to about 40° per cycle (table above). M4 always shifts forward, because it is faster forward. M3 shifts sometimes
  backward and sometimes forward (−41°, then +45°), which looks like gear play and sticking rather than a property of the motor.

**The real time between samples is 4–6 times longer than the 5 ms Timer.**
- **Measured:** the median is 20 ms in the step programs and 26–28 ms in the signal programs, which do more work per loop. The
  shortest interval is 13 ms, the longest 44–63 ms.
- **Why:** writing to the file in every loop takes most of the time.
- **How irregular:** fewer than 0.7 % of the intervals are more than twice the median.

What this changes:
1. One encoder count between two samples already means a speed of 1°/20 ms = 0.87 rad/s. That is 6 % of full speed, so the
   measured speed jumps in steps of that size.
2. The motor command is updated only every ~27 ms. At 5 Hz that is just 7 updates per period, and it adds a delay of about half a
   sample (~14 ms, about 25° at 5 Hz).
3. If we used 5 ms instead of the recorded times, every speed would come out 4–6 times too big. So the notebook always uses the
   recorded `time()` values.

**The battery did not have a measurable effect.**
- **Voltage readings:** only one reading per run: 11.2 V for all step series, 11.3 V for the square and chirp runs. The display
  shows only one decimal (0.1 V, about 0.9 %), so a voltage drop during one 63 s run is too small to see.
- **Repeated runs:** these give a better check. Run 2 divided by run 1 gives a speed ratio of 1.008 ± 0.039 for M4 and
  0.985 ± 0.020 for M3, and $T_m$ changes by less than 4 ms on average. So any battery effect is at most about 1–2 % of speed.

This is true at about 11.2 V. Close to TRIK's low-battery warning (10.9 V) the voltage falls faster. Also, the motor speed is not
simply proportional to PWM × battery voltage (section 6.2), so the $k_e$ values are only valid at this battery voltage.


**The encoder's whole-degree steps make the raw speed noisy.**
- **Raw speed:** at 60 % PWM in steady state, the speed from `np.gradient` jumps around with a standard deviation of 0.60–0.64 rad/s.
  This is close to the size of one encoder step (0.87 rad/s at 20 ms).
- **With averaging:** a 5-sample moving average brings this down to 0.21–0.32 rad/s. This is the average used everywhere in the
  notebook; it covers about 100 ms in the step files and about 135 ms in the signal files. A 15-sample average brings it down to
  0.06–0.09 rad/s, but it also blurs fast changes.

For odometry this is not a big problem, because odometry uses the angle itself, not the speed. The error is at most one count (1°)
per wheel per reading. A controller that needs the speed has to smooth it and accept some delay.


**The two wheels are not the same.** At the same PWM:
- **Speed:** M3 (right) is 12 % faster at +10 % and 19 % faster at −10 %, but only 2 % faster at +100 % and 6 % at −100 %.
- **Slope:** the 40–100 % slopes (and so $k_e$) differ by 10–13 %.
- **$T_m$:** M3 reacts more slowly (median about 0.15 s vs 0.11–0.14 s for M4).

So if both wheels get the same PWM, the robot will not drive straight; it will turn, most of all at low speed. The differences come
from normal manufacturing differences between motors and gearboxes, and they are several times bigger than the run-to-run
difference.

## 6. Report

### 6.2 Analysis

**How well does the first-order model with constant $T_m$ and $k_e$ describe the motor?**
Each single step looks like the model: the fitted curves miss the data by only about 1° (Figure 4.1). The problem is that the two
numbers do not stay constant:
- **Speed is not proportional to PWM.** Already at 10 % the wheel turns at 3.8–4.7 rad/s, about 30 % of full speed. Above 40 % the
  speed grows in a straight line (0.09–0.11 rad/s per %), but that line would reach zero at −32…−62 %, not at 0 (Figure 4.2).
- **$T_m$ changes with PWM.** It goes from 0.3–0.4 s at ±10 % down to 0.03–0.05 s at ±100 %. This happens in every run of both motors
  (Figure 4.3).

**Results:**
- With constant $k_e$ and $T_m$ (models A–C), the model matches the square wave only 54–82 % in speed, and between −127 % and +35 % in
  angle. At ±60 % the model predicts about 6 rad/s, but the real wheel turns at 9–10 rad/s, so the angle error grows every period.
- When the measured speed curve is used together with one fixed $T_m$ (model E), the square-wave match rises to 84 % (M3) and 86 % (M4)
  in speed, and 68 % and 82 % in angle. The 85 % target is reached only for the M4 speed.
- The chirp is harder (59–72 % in speed): above 2 Hz the wheel reacts about twice as strongly as the model with $T_m \approx 0.15$ s.

**Where the model is worst:**
- right after the direction changes, and at low PWM, where the wheel first has to break free;
- at fast frequencies (above 2 Hz);
- in the angle, where small forward/backward differences add up over time.

**Probable reasons:**
- **TRIK's "power %" is not a voltage.** By default the TRIK software changes the power value with a table before sending it to the
  motor driver (the `motor100` table: 10 % → 7 %, 60 % → 19 %, 90 % → 49 %, 100 % → 100 %). So
  $U = \mathrm{PWM}/100 \cdot U_{battery}$ is not the real motor voltage. This is how TRIK is set up by default; it was not checked on
  this robot.
- **Lightly loaded motors behave differently at low PWM.** With the wheels in the air, the current through the motor is not
  continuous at low PWM. This makes the motor faster and its reaction slower at low PWM, which fits both the steep start of the speed
  curve and the large $T_m$ there.

**Conclusion:** the basic shape of the model is right, but one $k_e$ is not enough. The model needs a measured speed curve (a lookup
table). It works for commands that change slowly (up to about 1–2 Hz), with $T_m \approx 0.13$–$0.15$ s. The chirp gives the same
value (0.13–0.17 s) independently.

**Forward vs backward rotation: are they the same motor? One parameter set or two?**
Not quite, and it depends on the motor:
- **M4** is 6–8 % faster forward than backward at every PWM (slope 10 % higher).
- **M3** differs by only 1–3 % in speed (slope 7 %).

Two runs of the same motor agree within 1–2 %, so the M4 difference is real. You can also see it in the square wave, where M4's angle
creeps forward every period (Figure 4.4). $T_m$ does not show a clear difference between directions; it changes much more with PWM.

**Recommendation:**
- **Without encoder feedback** (choosing a PWM for a wanted speed), use separate values for each wheel and each direction, best a speed
  table for each. Two straight lines are not enough either: model C (one line per direction) is still much worse than model E.
- **With encoder feedback**, one set of values is fine, because the feedback corrects the remaining few per cent.

**Which artifacts were found, how large are they, and which matter for odometry next week?**

| Artifact | How big | Important for odometry? |
|---|---|---|
| Speed not proportional to PWM, $T_m$ changes with PWM | 30 % of full speed already at 10 % PWM; $T_m$ 0.03–0.4 s | Not directly; but important for any controller that sets the speed through PWM |
| Forward/backward difference | 6–8 % (M4), 1–3 % (M3) | No: the encoders measure the real rotation |
| Left/right difference | 2–19 % in speed | No for odometry; yes for driving straight without feedback |
| Dead zone / breaking free | below 10 % once moving, about 9–15 % from rest | Very small movements may not happen, but the encoder still reports correctly |
| Saturation | none | — |
| Coasting after power-off | Wheel spins freely; slows down with about 0.24–0.30 s; 170–390° (about 11–25 cm) from full speed | **Yes:** keep reading the encoders after the stop command, because the robot rolls on |
| Irregular sampling | 20–28 ms instead of 5 ms, range 13–63 ms | **Yes:** always use the recorded time stamps, never a fixed time step |
| Encoder steps (quantisation) | 1° per count; 0.6–0.9 rad/s noise in the speed | **Yes, a little:** compute the position from angle differences, not from the noisy speed |
| Backlash | at most 1 sample / 1 count at reversals; up to ~40° shift per cycle in fast back-and-forth motion | Small in normal driving; matters when the robot changes direction often by small amounts |
| Battery drop | too small to measure (< 0.1 V); runs repeat within 1–2 % | No, as long as the values are measured near the voltage used |
| Zero drift | none (0 counts in 3 s) | Good: the encoders do not drift when the robot stands still |

The two biggest odometry errors could not be measured, because the wheels were in the air: **wheel slip** on the floor, and errors
in the **wheel radius and the distance between the wheels**. These should be calibrated on the floor next week.

**Which complex signal was chosen and what did it show that the step series did not?**
The main signal was a **square wave** (A = 60 %, P = 1 s, 8 s). The second was a **chirp** (A = 60 %, from 0.2 to 5 Hz in 10 s).

The square wave tested the model with 16 direction changes in one run. It showed that:
- the speed curve from the steps, with one fixed $T_m$, follows the real motion well (speed fit 84–86 %), while a constant $k_e$ does
  not (54–69 % for model A);
- the small forward/backward difference adds up into a drift of the angle;
- direction changes are not delayed by gear backlash by more than one sample.

The chirp showed things the steps cannot show, because it tests many speeds of change at once:
- the response gets about 4 times smaller from 0.6 to 4.5 Hz, and the delay grows to 72–78°;
- at slow frequencies (up to 2 Hz) the delay gives $T_m$ = 0.13–0.17 s, the same as the steps, but this time for a wheel that is
  already moving;
- above 2 Hz the motor is faster than any model with one fixed $T_m$;
- when the wheel only swings a few tens of degrees, it shifts by up to ~40° per cycle;
- at 5 Hz there are only 7 samples per period, so the sampling becomes a limit;
- its slow start from rest gave a direct estimate of the PWM at which the wheel breaks free (9–18 %).

