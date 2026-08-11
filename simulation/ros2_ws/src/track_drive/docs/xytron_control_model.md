# XYTRON Control Model

This note documents how the simulator converts `xycar_motor` command values into
vehicle motion. Use this when tuning `track_drive` speed and steering parameters,
especially the cone section.

## Command Units

The ROS command fields are normalized command values, not the physical state of
the vehicle.

```text
angle_cmd: -100.0 to 100.0
speed_cmd: -100.0 to 100.0
```

The simulator clamps both values before converting them.

## Speed Conversion

```text
targetSpeedMps = clamp(speed_cmd, -100, 100) / 100 * 50
targetSpeedMps = speed_cmd * 0.5
```

Examples:

```text
speed_cmd = 10.0    -> targetSpeedMps = 5.0 m/s   = 18.0 km/h
speed_cmd = 11.111 -> targetSpeedMps = 5.555 m/s = 20.0 km/h
```

## Steering Conversion

```text
targetSteeringDeg = clamp(angle_cmd, -100, 100) / 100 * 360
targetSteeringDeg = angle_cmd * 3.6
```

Sign convention:

```text
angle_cmd < 0 -> left turn
angle_cmd > 0 -> right turn
```

Examples:

```text
angle_cmd = -100 -> targetSteeringDeg = -360 deg -> max left
angle_cmd =  50  -> targetSteeringDeg =  180 deg -> right
```

## Actuator Lag

The command is a target. The actual simulator speed and steering approach that
target gradually.

Steering lag:

```text
steering_next = steering + 0.04 * (targetSteering - steering)
steering_next = 0.96 * steering + 0.04 * targetSteering
```

Speed lag during acceleration:

```text
speed_next = speed + 0.10 * (targetSpeed - speed)
speed_next = 0.90 * speed + 0.10 * targetSpeed
```

Speed lag during deceleration:

```text
speed_next = speed + 0.20 * (targetSpeed - speed)
speed_next = 0.80 * speed + 0.20 * targetSpeed
```

## Vehicle Motion Per Step

The simulator step time is `0.02 s`.

Heading update:

```text
yawRateDegPerSec = steeringDeg * speedMps / 50
yawDeltaDegPerStep = steeringDeg * speedMps / 50 * 0.02
```

Position update:

```text
x_next = x + speedMps * sin(headingDeg) * 0.02
z_next = z + speedMps * cos(headingDeg) * 0.02
```

Full step order:

```text
targetSteering = clamp(angle_cmd, -100, 100) * 3.6
targetSpeed = clamp(speed_cmd, -100, 100) * 0.5

if targetSpeed >= speed:
    speed += 0.10 * (targetSpeed - speed)
else:
    speed += 0.20 * (targetSpeed - speed)

steering += 0.04 * (targetSteering - steering)

heading += steering * speed / 50.0 * 0.02

x += speed * sin(radians(heading)) * 0.02
z += speed * cos(radians(heading)) * 0.02
```

## Cone Section Tuning Implication

Current cone launch parameters are command values, so their target physical
speeds are:

```text
cone_cruise_speed = 15.0 -> 7.50 m/s
cone_corner_speed = 10.5 -> 5.25 m/s
cone_min_speed    = 9.0  -> 4.50 m/s
cone_finish_speed = 9.0  -> 4.50 m/s
```

Raising cone speed should be done by changing the cone-only parameters, not the
lane-driving speed parameters:

```text
cone_cruise_speed
cone_corner_speed
cone_min_speed
cone_finish_speed
```

Because steering and speed both have lag, increasing cone speed also increases
corner overshoot risk. If the cone section starts cutting too wide or reacts too
late, tune cone speed and cone steering gains together instead of reusing lane
parameters.
