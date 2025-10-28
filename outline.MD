Desk-Cleaner Robot — AI Agent Outline
=========================================

Purpose
-------

An autonomous desk-cleaning robot that vacuums a flat tabletop using a simple SLAM/pose-graph and safe edge detection. Runs on a Raspberry Pi 5 with two NEMA17 steppers, TMC2209 drivers, a vacuum fan, and proximity sensors.

Hardware Summary
----------------

- Compute: Raspberry Pi 5, 5 V via XL4015 buck
- Drive: 2x NEMA17, TMC2209 drivers (VM from 4S battery)
- IMU: BNO085 (or MPU6050) for yaw; connected via I2C
- Proximity: 8x APDS-9960 down-facing around edges + 1x gesture/start sensor
- Vacuum: Neato 12 V BLDC fan (red=+12V, black=GND, blue=PWM, yellow=tach)
- Power: 4S Li-ion battery → 10 A main fuse → switch
  - 5 V rail (Pi, logic) via XL4015 (3 A fuse)
  - 12 V rail (fan) via XL4016 (5 A fuse)
- Wiring buses: 3.3 V bus from Pi for sensors/VIO; star ground

High-Level Software Architecture
--------------------------------

- Core loop @ 50–100 Hz:
  - Read IMU (yaw Δθ), motor encoders/angles (Δs)
  - Read proximity sensors (edge flags + analog proximity)
  - Update dead-reckoning pose (x, y, θ) and append to pose graph
  - Run state machine (Forward, Approach, Edge, OffsetTurn, Idle/End)
  - Control steppers (velocity or step targets) and fan PWM
- SLAM module:
  - Pose-graph data structure with nodes and constraints
  - Simple loop-closure hint via edge landmarks (optional)
- Planner:
  - Lane-based coverage with serpentine offsets
- Telemetry:
  - Log CSV: timestamp, x, y, θ, v, ω, sensor flags
  - Optional live Web UI (Flask) for map and state

State Machine (Simplified)
--------------------------

Init  -> WaitStart -> Forward -> [Approach?] -> Edge -> OffsetTurn -> Forward -> ... -> End

- WaitStart: gesture/clap to start; fan on
- Forward: v = v_cruise; if any sensors < TH_WARN -> Approach
- Approach: v = v_slow; if 2 adjacent sensors < TH_EDGE for N samples -> Edge; else back to Forward
- Edge: stop; add Edge landmark; back_off d_back while holding heading; go to OffsetTurn
- OffsetTurn: turn +90° (IMU); translate d_offset; turn +90° (IMU); flip turn_dir; go to Forward
- End: on time limit or user stop; save graph & logs

Dead Reckoning + Sensor Fusion
------------------------------

- Motion model (differential drive or step-based):
  - Δs from motor microsteps/angles → linear distance
  - Δθ from IMU yaw (complement with wheel-derived yaw if needed)
- Filter:
  - Simple complementary filter:
      θ_t = α*(θ_t-1 + Δθ_wheels) + (1-α)*θ_IMU
      s_t = Δs_motors
  - Optional EKF:
      State: [x, y, θ]; Inputs: [Δs, Δθ_IMU]; Process noise tuned empirically
- Pose update:
  x' = x + s_t *cos(θ)
  y' = y + s_t *sin(θ)
  θ' = wrap(θ + Δθ)

Pose Graph Formulation
----------------------

- Node i: pose (x_i, y_i, θ_i) at fixed step interval (e.g., every 2–5 cm or each state transition)
- Edges (constraints):
  - Odometry edge: between i and i+1 from (Δs, Δθ) with covariance Σ_odo
  - Landmark edge (edge detection): observation node L_k with range/bearing or a signed-distance to table boundary; large variance along boundary tangent
  - Optional soft prior on global yaw if magnetometer available
- Optimization:
  - Batch (Gauss-Newton/Levenberg-Marquardt) or periodic local window smoothing
  - Keep graph small (<= few hundred nodes) by marginalizing old nodes
- Outputs:
  - Best estimate of trajectory; boundary line estimates; coverage heat map

Edge Detection & Safety
-----------------------

- Early warning threshold TH_WARN to slow down
- Confirmed edge when 2 adjacent sensors < TH_EDGE for N samples (debounce)
- On Edge:
  - Stop; append Edge landmark at current pose
  - Back-off d_back (e.g., 3–4 cm) with yaw tolerance |Δθ|<2°
  - OffsetTurn sequence with IMU-locked 90° turns and d_offset advance

Coverage Strategy (Desk Cleaning)
---------------------------------

- Start at one corner or any pose
- Serpentine lanes parallel to the longest desk dimension
- Lane width ~ tool width minus small overlap (e.g., 10–15%)
- After each Edge event: perform OffsetTurn sequence to enter next lane
- Time limit or coverage fraction target ends run

Data Structures
---------------

PoseNode:
  id, t, x, y, θ, state_tag
OdoEdge:
  from_id, to_id, Δs, Δθ, Σ_odo
LandmarkNode (Edge):
  id, type="edge", pose_at_obs, normal_dir
LandmarkEdge:
  node_id, lm_id, residual, Σ_lm

Control Interfaces
------------------

- Motor driver API: set_velocity(v_left, v_right) or step_to(target)
- IMU API: read_yaw(), bias_calibrate()
- Sensors API: read_edge_flags(), read_gesture()
- Fan API: set_pwm(percent), tach_read()

Parameters (tune once)
----------------------

- v_cruise, v_slow
- TH_WARN, TH_EDGE, N_debounce
- d_back, d_offset
- α (complementary filter), Σ_odo, Σ_lm
- Node spacing for graph (meters or state transitions)
- Time limit (e.g., 120 s)

Testing & Validation
--------------------

- Bench test: verify IMU bias/scale; i2cdetect; stepper current tuning; fan PWM
- Straight-line odometry: 1 m runs; measure drift; set α and Σ_odo
- Edge trigger test: place tape line as fake edge; adjust TH_* and N
- Full sweep test: validate serpentine coverage and no re-trigger loops
- Log replay: run pose-graph optimizer offline; compare to ground truth marks

Safety
------

- Main 10 A fuse on battery +; 3 A fuse on 5 V; 5 A on 12 V
- Emergency stop: power switch reachable; software stop hotkey
- Brown-out watch: Pi voltage monitor; throttle fan on low voltage
- Thermal: drivers heatsinked; fan airflow near drivers

Minimal Code Skeleton (Pseudo)
------------------------------

loop at 50 Hz:
  imu = read_imu()
  odo = read_wheels()
  pose = update_pose(pose, imu, odo)         # dead reckoning
  add_pose_node_if_needed(pose)
  edges = read_edge_sensors()

  if state == FORWARD:
    set_speed(v_cruise)
    if any(edges < TH_WARN): state = APPROACH

  elif state == APPROACH:
    set_speed(v_slow)
    if confirmed_edge(edges): state = EDGE
    elif all_clear(edges): state = FORWARD

  elif state == EDGE:
    stop_motors()
    add_edge_landmark(pose)
    back_off(d_back, keep_heading=True)
    offset_turn(90, dir=turn_dir)
    forward(d_offset)
    offset_turn(90, dir=turn_dir)
    turn_dir = flip(turn_dir)
    state = FORWARD

  if time_elapsed > LIMIT: state = END

  log(imu, odo, pose, state)

End:
  save_graph()
  stop_motors(); fan_off()

Roadmap Ideas
-------------

- Add TCA9548A I2C mux for many APDS-9960s (address conflicts)
- Hall encoders on wheel shafts for higher-quality odometry
- Occupancy line model of desk boundary from repeated edge landmarks
- Simple loop-closure when revisiting start lane
- Web UI with live map and buttons (start/stop, lane width, speed)
