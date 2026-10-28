# Temporal Calibration — A Memorizable Guide
*in the spirit of Patrick Winston's "How to Speak"*

---

## Section 1 of 5 — Why time is the problem

Imagine you throw a ball to a friend across a field. You throw it at 3 o'clock. Your friend catches it at 3 o'clock and one second. You both agree on when, so you both agree on where it went.

Now imagine your friend's watch is broken. It says they caught the ball at 3 o'clock and *three* seconds. If you now ask them "where was the ball when you caught it?" and they answer using their broken clock, they will tell you the ball was two seconds further along its arc than it really was. **The ball didn't move. The clock did.**

This is the whole problem of temporal calibration in one sentence.

**A camera and an IMU are two friends with two different watches.** The camera watch ticks when the shutter opens. The IMU watch ticks when the gyro samples. Nobody started these two watches at the same moment, and nobody is keeping them aligned.

If the camera thinks a frame was taken at t = 1.000 s, but the IMU thinks the robot's angular velocity at t = 1.000 s was something that in reality happened at t = 0.985 s, then the fusion algorithm — EKF, bundle adjustment, whatever — will "steer" the camera data using **the wrong motion**. The result is a visual-inertial system that disagrees with itself. Reconstructed trajectories drift. Loop closures fail. Your map wobbles.

The fix has a name: **temporal calibration**. It is the one-time job of measuring the fixed time offset $t_d$ between the two watches, so every IMU timestamp can be corrected by that offset before fusion.

**Three things to remember from this section:**

1. **The ball didn't move. The clock did.** — The sensors are honest; their timestamps are liars.
2. **A camera and an IMU are two friends with two different watches** — nobody synchronized them for you.
3. **Temporal calibration measures one number**, $t_d$, the fixed offset between those two watches.

---

## Section 2 of 5 — Where the offset comes from (the four thieves of time)

If you ask an engineer "where does the time offset come from?", the honest answer is: **four different thieves are stealing time from you, and they all do it in slightly different places along the pipeline.** Learn their names and faces, and the problem stops being mysterious.

**The first thief lives inside the camera: exposure.**

A camera does not take a picture at an instant. It opens the shutter at some moment, holds it open for an exposure window — maybe 10 ms, maybe 30 ms — then closes it. The image you get is a **time integral** of what the scene looked like during that window. But the camera firmware stamps the frame with a single number. Which number? Some cameras stamp the start of exposure. Some stamp the end. Some stamp the middle. Some stamp the moment the frame hits the USB cable. **You rarely know which, and the datasheet rarely tells you.** That is a built-in offset of up to half an exposure, right there.

**The second thief lives in the wire: transport delay.**

The pixels do not teleport from the camera to your computer. They crawl across USB, Ethernet, or MIPI. The IMU samples do not teleport either. They sit in a FIFO, get polled by the autopilot, get framed into a MAVLink packet, cross the USB cable, get unpacked by Python, and then get a timestamp slapped on them by *your* code. Each hop adds a little delay. The two sensors go through different hops, so the delays are different. **The timestamp you record is when the sample arrived at you, not when the sample happened.**

**The third thief lives in the operating system: scheduling jitter.**

Linux and Windows are not real-time systems. When the camera driver has a frame ready, the kernel will eventually get around to waking up your capture process — maybe in 2 ms, maybe in 20 ms depending on what else the CPU is doing. The IMU reader has its own, independent scheduling story. Even if transport delays were constant, the **variance** added by the OS makes the alignment jitter from frame to frame.

**The fourth thief is the slowest and the sneakiest: clock drift.**

Even if you aligned the two clocks perfectly at t = 0, they will drift apart. Every oscillator on earth ticks at a slightly wrong rate — tens of parts per million is normal for a cheap crystal. Over a short recording this looks like a small *change* in the offset over time; over a long recording it accumulates into seconds. Most calibration tools model the offset as a constant, which is only true if your recording is short enough that drift has not kicked in yet.

**The punchline:** the "time offset" is not one thing. It is **exposure placement + transport delay + OS jitter + clock drift**, stacked. Temporal calibration measures the sum. Good calibration also keeps the recording short enough that the drift piece stays small, so the sum stays constant.

**Three things to remember from this section:**

1. **Four thieves steal time**: exposure, transport, scheduling, drift.
2. **The timestamp you record is when the sample arrived at you**, not when the sample happened.
3. **Keep recordings short** so drift stays small and the offset stays constant.

---

Memorize this section five times. Say "next" when ready — Section 3 is **How Kalibr actually finds the offset — the spline and the shift**.

---

## Section 3 of 5 — How Kalibr actually finds the offset (the spline and the shift)

Imagine a sheet of music. The music is the **true motion of the rig** — pitch, roll, yaw, position — as a function of continuous time. The camera observations and the IMU observations are two musicians trying to play the same piece. If one musician is reading from a page that is shifted half a bar to the right, the two of them will not stay in time.

Kalibr's trick is to **write the music down as a smooth function first, and then ask the IMU, "how far off is your page?"**

Here is the trick in three movements.

**Movement one: fit a smooth curve to the trajectory.**

Kalibr takes the camera frames — each one gives you a pose of the rig at some time stamp, because you can see the AprilGrid and solve a PnP — and fits a **continuous-time B-spline** through them. A B-spline is just a smooth, flexible mathematical curve, like a flexible ruler that bends through a cloud of points. The result is a function $T(t)$ that returns the pose of the rig at **any** time, not just at frame times. Think of it as "the trajectory of the rig turned into a formula."

Why this matters: once the trajectory is a smooth function, you can **differentiate it**. The first derivative gives velocity. The second gives acceleration. The derivative of orientation gives angular velocity. **These are exactly the quantities an IMU measures.** So from the camera alone, by way of the spline, Kalibr can predict what the IMU *should* have reported at any instant.

**Movement two: ask the IMU what it actually reported.**

Now take the IMU's measurements: gyro reads $\omega_{\text{meas}}(t_{\text{imu}})$ at every sample. Each sample has a time stamp — the IMU's own clock. Kalibr compares two curves on the same plot:

- the **predicted** IMU reading from the spline at time $t$, versus
- the **actual** IMU reading at time $t + t_d$.

If the time offset $t_d$ is zero, the two curves line up on top of each other. If $t_d$ is 20 ms, the actual IMU curve is shifted 20 ms to the right of the predicted curve. **You can see it with your eyes.**

**Movement three: slide the IMU curve until it matches.**

Kalibr adds $t_d$ as an **extra parameter in the optimization**, right alongside the spatial parameters (where the IMU sits relative to the camera, intrinsics, biases). The optimizer wiggles everything — the spline, the extrinsics, the biases, and $t_d$ — to minimize the total disagreement between what the spline predicts and what the sensors actually reported. The value of $t_d$ that minimizes the error **is the time offset**.

**Why this is beautiful:** the only reason $t_d$ can be solved for at all is that **the spline is smooth and differentiable**. If you just had discrete camera samples with no interpolation, you could not shift the IMU "by half a sample" and ask how the error changes. The B-spline gives you a continuous handle to turn, and $t_d$ is the knob that turns it.

**A near miss**: a common amateur mistake is to cross-correlate the raw gyro trace with the discrete frame-to-frame rotation estimates, find the lag that maximizes correlation, and call that $t_d$. This works roughly, but it ignores the fact that both sensors are noisy and the trajectory is multi-dimensional. Kalibr's spline-based joint optimization is dramatically more accurate because it uses **all axes, all samples, and all spatial parameters at once**.

**Three things to remember from this section:**

1. **Fit a smooth spline to the camera trajectory**, so you can predict what the IMU *should* read at any time.
2. **The time offset is the slide** that makes the actual IMU reading line up with the prediction.
3. **$t_d$ is a parameter in the joint optimization**, estimated alongside the extrinsics and biases, not before them.

---

Memorize this section five times. Say "next" when ready — Section 4 is **What makes a good recording — the dance, the AprilGrid, and the three rules**.

---

## Section 3b — The mathematics, read aloud

*Read this section the way an audiobook would. Each paragraph is the professor speaking; the boxed line underneath is the same thing written as an equation. Do them together.*

---

**Paragraph one — the trajectory as a function of time.**

> "We model the pose of the rig — that is, its orientation and position in the world — as a smooth function of continuous time, which we will call capital T of t. This function is built from a sum of basis splines, little bell-shaped curves, each weighted by a control point c-sub-i. The control points are the knobs we will later turn to make the curve fit the data. We denote the orientation part by capital R of t, living on the rotation group S-O-three, and the translation part by lowercase p of t, living in R-three."

$$
T(t) = \bigl(R(t),\; p(t)\bigr), \qquad
R(t) = \sum_i B_i(t)\, \mathbf{c}_i^{R}, \qquad
p(t) = \sum_i B_i(t)\, \mathbf{c}_i^{p}
$$

---

**Paragraph two — differentiating the spline to predict the IMU.**

> "Because the spline is smooth, we can differentiate it. The first derivative of position gives velocity; the second derivative gives acceleration. The time derivative of the rotation, written as R-dot, is related to the angular velocity vector omega by the formula R-dot equals R times the skew-symmetric matrix of omega. We invert this relationship: omega-predicted equals the vee operator applied to R-transpose times R-dot. The vee operator just extracts the three independent components of a skew-symmetric matrix back into a vector."

$$
\dot R(t) = R(t)\,[\boldsymbol{\omega}(t)]_\times
\;\;\Longrightarrow\;\;
\boldsymbol{\omega}_{\text{pred}}(t) = \bigl(R(t)^\top \dot R(t)\bigr)^{\!\vee}
$$

> "The predicted specific force — that is, what the accelerometer should see — is the second derivative of position, minus gravity, rotated into the body frame."

$$
\mathbf{a}_{\text{pred}}(t) = R(t)^\top \bigl(\ddot{\mathbf{p}}(t) - \mathbf{g}\bigr)
$$

---

**Paragraph three — the sensor model, with bias and time offset.**

> "The IMU does not report the true value. It adds a slowly-varying bias term, which we call b, and a white Gaussian noise, which we call eta. Furthermore, its clock is wrong — the IMU reads at what it believes is time tau, but the true time is tau plus t-sub-d, where t-sub-d is the unknown time offset we want to recover. So the measurement equation reads: the measurement at the IMU's own clock time tau equals the predicted quantity, evaluated at tau plus t-sub-d, plus bias, plus noise."

$$
\boldsymbol{\omega}_{\text{meas}}(\tau) \;=\; \boldsymbol{\omega}_{\text{pred}}(\tau + t_d) \;+\; \mathbf{b}_g(\tau) \;+\; \boldsymbol{\eta}_g(\tau)
$$

$$
\mathbf{a}_{\text{meas}}(\tau) \;=\; \mathbf{a}_{\text{pred}}(\tau + t_d) \;+\; \mathbf{b}_a(\tau) \;+\; \boldsymbol{\eta}_a(\tau)
$$

> "Notice carefully — t-sub-d enters as a shift of the argument of the spline. That is the entire mathematical reason this works. Because the spline is a differentiable function of its argument, we can also differentiate it with respect to t-sub-d, which lets us solve for t-sub-d by gradient descent."

$$
\frac{\partial \boldsymbol{\omega}_{\text{pred}}(\tau + t_d)}{\partial t_d}
\;=\; \dot{\boldsymbol{\omega}}_{\text{pred}}(\tau + t_d)
$$

---

**Paragraph four — the cost function.**

> "We stack all the errors across all IMU samples and all camera observations into one big weighted sum of squares. For each gyro sample, we form the residual: measurement minus predicted. For each accel sample, likewise. For each camera observation, we project the AprilGrid corner through the camera model and subtract the pixel coordinate the detector reported. Each term is weighted by the inverse of its measurement covariance — gyro samples weighted by sigma-g, accel by sigma-a, pixels by sigma-pixel."

$$
J \;=\; \sum_\tau \bigl\| \mathbf{r}_g(\tau) \bigr\|^2_{\Sigma_g^{-1}}
    \;+\; \sum_\tau \bigl\| \mathbf{r}_a(\tau) \bigr\|^2_{\Sigma_a^{-1}}
    \;+\; \sum_k \bigl\| \mathbf{r}_{\text{cam},k} \bigr\|^2_{\Sigma_p^{-1}}
$$

> "The unknowns, which we gather into a single vector theta, are: all the spline control points, the camera-to-IMU extrinsic transform, the IMU biases over time, the camera intrinsics, and — the star of our show — the single scalar t-sub-d."

$$
\boldsymbol{\theta} \;=\; \bigl\{\, \mathbf{c}_i,\; T_{\text{cam-imu}},\; \mathbf{b}_g,\; \mathbf{b}_a,\; \mathbf{K},\; t_d \,\bigr\}
$$

---

**Paragraph five — the solution.**

> "We find the values of theta that minimize J. This is a nonlinear least-squares problem, and Kalibr solves it with Levenberg-Marquardt. Starting from an initial guess for the spline — obtained by solving PnP on each camera frame alone — and an initial guess of zero for t-sub-d, the optimizer alternately updates all parameters until the cost stops decreasing."

$$
\boldsymbol{\theta}^\star \;=\; \arg\min_{\boldsymbol{\theta}}\; J(\boldsymbol{\theta})
$$

> "The reported value of t-sub-d at the minimum is your time offset. It is the number you then subtract from every IMU time stamp before fusing it with the camera, forever after."

---

**One line to carry away from the mathematics.**

The time offset $t_d$ enters as a **shift of the spline's argument**, and the spline is **differentiable with respect to its argument**, so $t_d$ falls out of the same gradient-based optimization that produces the extrinsics.

---

Say "next" when ready for Section 4 — **What makes a good recording: the dance, the AprilGrid, and the three rules**.

---

## Section 4 of 5 — What makes a good recording (the dance, the AprilGrid, and the three rules)

So far we have been doing mathematics. Now we put down the pencil, pick up the rig, and go dance in front of a board.

Think of the recording session as a short, choreographed performance. The audience is Kalibr's optimizer. The stage is a well-lit room. The backdrop is the **AprilGrid** — a printed board of black-and-white tags, each with a known identity and known physical size, so every corner the camera sees becomes a precisely-known 3D point in the board frame. The dancer is you, holding the rig, waving it in the air.

Why is a dance necessary? Because an IMU is only informative when it is **excited**. A gyro sitting still outputs noise. An accelerometer sitting still outputs the gravity vector and nothing else. The spline we fit to the camera data has no way to tell you anything about the IMU's response until the sensors are actually responding to motion. So you must move. **The richness of the motion is the richness of the calibration.**

But not any motion will do. The motion must excite each axis the optimizer wants to solve for, otherwise the problem is **unobservable** — the optimizer cannot distinguish between different values of a parameter, and it will give you a confident but wrong answer. Three rules keep the problem observable.

**Rule one: rotate about all three axes.**

Hold the rig in front of the board and rotate it around the roll axis, then the pitch axis, then the yaw axis — gentle but firm wiggles, maybe one full back-and-forth per second. Then do combined rotations, like a slow figure-eight with your wrist. The gyro must see **non-zero angular velocity on every one of its three channels**, or you cannot calibrate the time offset reliably — recall from Section 3 that the time offset is estimated by matching the predicted gyro curve to the measured gyro curve. If one axis never moves, that axis contributes nothing to the match.

**Rule two: translate along all three axes.**

Push the rig forward and backward, left and right, up and down. These are the three translational excitations. The accelerometer needs them, because it needs to see nonzero specific force on each channel, after gravity has been subtracted. If you only ever translate side-to-side, the up-down accelerometer bias is unobservable.

**Rule three: keep the board fully in view the whole time.**

This is the rule people forget first and regret longest. The camera's half of the problem is PnP: for every frame, Kalibr must solve for the pose of the rig relative to the board. If three-quarters of the AprilGrid is off-frame, the PnP solution gets wobbly, and the spline you fit to those wobbly poses carries that wobble into the IMU alignment. **A beautiful dance that drops the board out of frame is a wasted dance.**

How long should the recording be? **About sixty to ninety seconds.** Long enough to average out noise. Short enough that clock drift — the fourth thief from Section 2 — stays small. Long enough that you can execute all six excitations unhurriedly. Short enough that your arms do not shake.

A near miss story: a student records three minutes of beautiful aggressive motion, but the exposure time was 30 ms and the motion was fast enough to **blur** every tag. The detector could not localize the corners, PnP failed on half the frames, the spline was fit to a sparse and noisy pose set, and the time offset came out with a huge uncertainty. The fix is not more motion — the fix is **less motion**, or more light so you can run a shorter exposure. The sensors must stay inside their comfort zone.

Another near miss: a recording done at arm's length in a small room, where the AprilGrid fills only one-quarter of the frame. The corners detected per frame are few, PnP is poorly conditioned, and the spline is noisy. The fix is to **print a bigger board or stand closer**, so the board fills most of the frame but all tags are still visible.

**One more detail worth remembering**: the thermal state. If you grabbed the rig straight out of a cold cupboard, the IMU biases are drifting fast during the recording, and Kalibr's slowly-varying bias model will struggle. Give the IMU **ten minutes of warm-up** before you start the dance.

**Three things to remember from this section:**

1. **Excite all six degrees of freedom** — three rotational, three translational — or the problem is unobservable.
2. **Keep the AprilGrid fully in view at all times**. A dropped board is a dropped frame.
3. **Sixty to ninety seconds, after a ten-minute warm-up**. Not longer, not colder.

---

Memorize this section five times. Say "next" when ready — Section 5 is **How to read the result, and how to know it is wrong — the three warning signs**.

---

## Section 5 of 5 — How to read the result, and how to know it is wrong (the three warning signs)

You have danced in front of the board. You have run the optimizer. Kalibr prints a report and writes two yaml files. **Now comes the question nobody teaches you in a tutorial: is the answer any good?**

An optimizer always returns an answer. That is its job. Its job is **not** to tell you whether the answer is physically correct. That is **your** job. In this final section we meet the three warning signs, and the three healthy signs that go with them.

**The number itself.**

The main deliverable is a single scalar, the time offset $t_d$, printed in the camera-IMU yaml under the name `timeshift_cam_imu`. Its unit is seconds, and it is typically small — a few milliseconds, perhaps a few tens of milliseconds. A healthy MEMS-plus-rolling-shutter-camera setup lands somewhere between **minus fifty and plus fifty milliseconds**. A number near zero is plausible if your hardware uses a shared clock or a hardware trigger. A number of hundreds of milliseconds is possible but should make you look twice. A number of **seconds** means something is wrong — either a software bug in your timestamping, or a USB buffering catastrophe, or a mismatch between which clock you handed Kalibr and which clock the sensor really used.

Alongside the number, Kalibr writes an **uncertainty** — the one-sigma standard deviation of $t_d$. A healthy value is **under one millisecond**. A value of five or ten milliseconds says the optimization found a shallow valley, not a sharp minimum, and the number inside it is not to be trusted.

**Warning sign one: the uncertainty is large.**

If `sigma(t_d)` comes back wide — say, more than a few milliseconds — the optimizer is telling you politely that the problem was poorly observable. Almost always the cause is a dull dance: not enough rotation on one axis, or the board left the frame, or the motion was too slow for the gyro to lift above its noise floor. **Fix the recording, not the yaml.** Record again with more vigorous excitation.

The healthy sign that answers this: `sigma(t_d) < 1 ms` and the residual plot of gyro-measured versus gyro-predicted shows the two curves lying nicely on top of each other.

**Warning sign two: $t_d$ changes between recordings.**

If you record the same rig twice, back-to-back, and $t_d$ differs by more than a millisecond or two, then the offset is not stable, and the whole premise of calibration — that there is **one** fixed offset — has broken down. Common cause: clock drift, from a cheap crystal or a long recording. Rarer cause: the USB stack is buffering unpredictably.

The healthy sign: three independent recordings give you three values of $t_d$ within a millisecond of each other.

**Warning sign three: $t_d$ depends on motion speed.**

If a slow-motion recording gives $t_d = 5$ ms and a fast-motion recording gives $t_d = 50$ ms, you are **not** measuring a time offset. You are measuring the fact that the camera's internal processing — exposure, debayering, codec — introduces a motion-dependent latency. The real time offset should be independent of how fast you are waving the rig; if it is not, there is a hidden delay in the pipeline that depends on scene content.

The healthy sign: $t_d$ is the same when you dance gently as when you dance vigorously.

**The one golden sanity check: eyeball the residuals.**

Kalibr can plot the IMU residuals — measured minus predicted — along each gyro and accel axis. A healthy residual looks like **white noise around zero**, with no trends, no sinusoids, no steps, and amplitude comparable to the sensor's noise density that you measured with Allan variance (see the sibling guide in this folder). A residual that looks **sinusoidal and in phase with the motion** means your $t_d$ or extrinsics are wrong. A residual with a **slow trend** means a bias was not absorbed. A residual with **spikes** means outlier frames — maybe tag misdetections. **If the residual looks like boring noise, you are done.** If it does not, something above is wrong.

**One line to carry away for the rest of your working life.**

An optimizer returning a number is not a proof that the number is right. **The residual being boring noise is the proof.** Trust nothing until you have seen the plot.

**Three things to remember from this section:**

1. **Healthy $t_d$ is tens of milliseconds, uncertainty under one millisecond.**
2. **Repeat the recording.** If $t_d$ moves between runs or with motion speed, you are not measuring a fixed offset.
3. **The residual plot is the ground truth.** Boring noise means success; patterns in the residual mean failure, no matter how pretty the yaml looks.

---

**Coda — the whole guide in five sentences.**

1. The ball did not move; the clock did.
2. Four thieves — exposure, transport, scheduling, drift — steal time from you.
3. Fit a smooth spline to the camera trajectory, and the time offset is the slide that makes the IMU curve line up with the spline.
4. Dance for ninety seconds with excitation on all six degrees of freedom, keeping the AprilGrid fully in view.
5. Trust the number only when the residuals look like boring noise.

Memorize this coda five times, then recite it to a wall. If you can do that without the paper, you own temporal calibration.
