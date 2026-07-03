# Physics & Control Model

Here is what actually happens, once a second, inside `runner.py`.
<img width="1920" height="995" alt="image" src="https://github.com/user-attachments/assets/bd0f1a2a-2633-44ce-bea4-25fe704d63b9" />

<img width="1900" height="985" alt="image" src="https://github.com/user-attachments/assets/001cfee4-eab0-441e-8dda-1e9eab619562" />



---

## 0. The drive cycle

Before any of the physics runs, here's `build_drive_cycle()` - lookup table of target speeds, one per tick, built once before the loop even starts:

```
t <  100   → target speed = 0    (stopped)
t <  250   → target speed = 30   (accelerate, cruise)
t <  400   → target speed = 50   (accelerate again)
t <  500   → target speed = 0    (decelerate to a stop)
t <  700   → target speed = 80   (hard acceleration)
t <  850   → target speed = 40   (decelerate, settle)
else       → target speed = 0    (stop)
```

Alongside it, `build_road_profile()` does the same thing for terrain, a separate lookup table saying what the road grade is at each second, independent of speed:

```
t <  200   → grade = 0.0°    (flat)
t <  450   → grade = 4.5°    (climbing)
t <  700   → grade = -2.0°   (descending)
t <  850   → grade = 1.5°    (a smaller rise)
else       → grade = 0.0°    (flat)
```

These two profiles are what turn the simulation from "a car that can move" into "a car doing something specific": a stop-start city segment, a climb, a fast stretch, a descent, a settle. Neither one reacts to the other or to anything the physics engine is doing. They're both just fixed scripts.
The PID controller's whole job, every tick, is to chase whatever number `target_speed[t]` says it should be chasing.

Limitation here: real drive cycles, WLTP, city-highway blends, anything based on actual recorded driving, ramp smoothly between speeds. This one jumps. Instantly, at the boundary of each segment, target speed steps from one value to another with no transition at all. That's a deliberate simplification for better PID results.

---

## 1. The controller

The simulation always knows two numbers: where the car should be, and where it is. Everything starts with the gap between them.

```
error(t) = target_speed(t) - speed(t)
```

This is the same closed-loop idea behind real cruise control, thermostats, drone stabilization: measure the gap, act, measure again. So the controller keeps these three questions running:

**P, how big is the gap right now.**

```
P = Kp * error        (Kp = 2.0)
```

Bigger gap, harder push. On its own, this term has a problem: the car oscillates around the target forever, overshooting and undershooting It needs the other two.

**I, how long has this gap been sticking around.** The error, added up over time:

```
integral(t) = clip( integral(t-1) + error(t) * dt,  -integral_max,  +integral_max )
I = Ki * integral        (Ki = 0.02)
```

This is what corrects the stubborn, lingering error a P-term alone can't close. But it has a failure mode worth naming: *integral windup*. If the car sits stopped for a long stretch, waiting at the start of a drive cycle, say, the integral just keeps accumulating in the background, quietly building up a debt. Then the moment the target speed finally moves, all that stored-up error slams through at once as a jerk of throttle nobody asked for. The `clip()` on `integral_max` exists specifically to stop that debt from growing past the point of usefulness.

**D, how fast the gap is changing.** Not where the error is, where it's headed:

```
derivative(t) = ( error(t) - error(t-1) ) / dt
D = Kd * derivative        (Kd = 0.5)
```

If speed is closing in on the target fast, the derivative senses that and eases off before overshoot happens, a damper, smoothing out what would otherwise be an oscillation. 

All three combine into one number:

```
throttle(t) = clip( Kp*error(t) + Ki*integral(t) + Kd*derivative(t),  0,  100 )
```
Without the clip, the math could hand back a negative throttle, or something past 100%, and neither of those mean anything.

I took AI suggestions to choose gains. 

---

## 2. Throttle becomes motion

Throttle is a percentage. 

```
motor_torque = max_torque * (throttle / 100)
```

That torque passes through the gearbox, multiplied up, the way gears do:

```
wheel_torque = motor_torque * gear_ratio
```

And torque at a wheel of a given radius becomes force at the point where rubber actually meets road:

```
motor_force = wheel_torque / wheel_radius
```

Really important number here.

---

## 3. Forces



**Rolling resistance**, the small, constant tax of tyres deforming against the road:

```
rolling_resistance = Crr * mass * g * cos(theta)
```

**Drag**, air, refusing to get out of the way. 
```
drag_force = 0.5 * air_density * Cd * frontal_area * (speed / 3.6)^2
```

**Grade**, gravity, pulling the car back down whatever hill it's trying to climb:

```
grade_force = mass * g * sin(theta)
```

At 4.5° incline, the grade force is significant, not negligible.

Using `sin(4.5°) ≈ 0.078`, a 1500 kg vehicle experiences about **1147 N** of additional resisting force just from gravity along the slope. That load persists throughout the climb and is a meaningful portion of total available traction and motor output.

This comes directly from splitting gravity into components along and perpendicular to the slope: `sin(θ)` gives the downslope force (grade resistance), while `cos(θ)` affects normal force and thus rolling resistance. Both depend on the same angle but impact the system differently.


---

## 4. Regen

```
if throttle < regen_throttle_threshold and speed > regen_speed_threshold:
    regen_current = max( -regen_gain * speed,  -regen_current_limit )
    motor_force  -= regen_brake_force
```

The current goes negative, flowing into the battery instead of out. A small kindness, built into the coasting.

---

## 5. Newton and His Laws 

f = ma

```
net_force = motor_force - rolling_resistance - drag_force - grade_force
acceleration = net_force / mass
```

Speed updates:

```
speed_ms(t) = max( speed_ms(t-1) + acceleration * dt,  0 )
speed_kmh   = speed_ms * 3.6
```

And distance just keeps counting:

```
distance_km += speed_kmh * dt / 3600
```

---

## 6. RPM and Efficiency

Speed tells you RPM:

```
wheel_rpm = ( speed_ms / (2*pi*wheel_radius) ) * 60
rpm = max( wheel_rpm * gear_ratio + noise,  0 )
```

But RPM alone doesn't tell you efficiency. Real motors have a sweet spot, a place where they run cleanest, and get worse the further you push them from it, in either direction:

```
rpm_factor    = max( 0.5, 1 - ((rpm - peak_rpm) / 8000)^2 )
torque_factor = max( 0.6, 1 - ((motor_torque - 150) / 350)^2 )
motor_eff     = clip( base_motor_efficiency * rpm_factor * torque_factor,  0.70,  0.94 )
```

Not a perfect model of a real motor curve. 

---

## 7. Electrical Power

Mechanical power, first: torque doing its work at a given angular speed:

```
angular_velocity = rpm * 2*pi / 60
p_motor_kW       = motor_torque * angular_velocity / 1000
```

Mechanical Energy equivalent to Electrical Energy

```
if p_motor_kW > 0:   # driving: losses stack, so more is drawn than the motor alone would suggest
    power = (p_motor_kW / motor_eff) / battery_efficiency
else:                 # regen: losses stack the other way, so less comes back than was captured
    power = p_motor_kW * motor_eff * battery_efficiency
```

---

## 8. The battery

Current isn't just the load from driving. Electronics in the car never sleep.

```
current = quiescent_current + (throttle*0.8 + speed*0.15) + regen_current + noise
```

Voltage rises with how full the battery is:

```
ocv = 320 + 90 * (soc / 100)
```

Internal Resistance:

```
voltage = ocv - current * battery_resistance
```

And charge just drains, irrespective of how much power was spent:

```
battery_energy = clip( battery_energy - power*dt/3600,  0,  battery_capacity_kWh )
soc = (battery_energy / battery_capacity_kWh) * 100
```

---

## 9. Heat

Current through resistance makes heat. V=IR:

```
heat_generation = current^2 * battery_resistance
```

The pack warms from that heat, and cools backs through environmental contact:

```
temp += heat_generation * thermal_mass_factor
temp -= (temp - ambient_temp) * cooling_rate
temp  = max(temp, ambient_temp)
```

`thermal_mass_factor` decides how quickly a spike in current shows up as a spike in temperature. `cooling_rate` decides how quickly the pack lets that heat go, once the current backs off. 

---

## 10. Kinetic Energy


```
kinetic_energy_J = 0.5 * mass * speed_ms^2
```

---

## Summary

```
look up target speed, look up road grade
     → gap → throttle
     → torque → force at the wheels
     → drag, rolling resistance, grade, pushing back
     → regen, if coasting
     → net force → acceleration → speed → distance
     → RPM → motor efficiency
     → power the battery actually pays
     → current, voltage, charge
     → heat → temperature
```

Then it writes the row, and does it again. A thousand times.
