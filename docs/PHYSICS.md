# Physics & Control Model

Here is what actually happens, once a second, inside `runner.py`.

Not a summary. The real thing — every equation, in the order the code runs it, explained the way I'd explain it to you if we were sitting with the code open between us.

---

## 1. The controller: deciding how hard to push

The simulation always knows two numbers — where the car should be, and where it is. Everything starts with the gap between them.

```
error(t) = target_speed(t) - speed(t)
```

That gap alone isn't enough to act on well. This is the same closed-loop idea behind real cruise control, thermostats, drone stabilization — measure the gap, act, measure again. So the controller keeps three running impressions of that gap:

**P — how big is the gap right now.**

```
P = Kp * error        (Kp = 2.0)
```

Bigger gap, harder push. Reacts to the present, nothing else. On its own, this term has a problem: the car oscillates around the target forever, overshooting and undershooting, never quite settling. It needs the other two.

**I — how long has this gap been sticking around.** The error, added up over time:

```
integral(t) = clip( integral(t-1) + error(t) * dt,  -integral_max,  +integral_max )
I = Ki * integral        (Ki = 0.02)
```

This is what corrects the stubborn, lingering error a P-term alone can't close. But it has a failure mode worth naming: *integral windup*. If the car sits stopped for a long stretch — waiting at the start of a drive cycle, say — the integral just keeps accumulating in the background, quietly building up a debt. Then the moment the target speed finally moves, all that stored-up error slams through at once as a jerk of throttle nobody asked for. The `clip()` on `integral_max` exists specifically to stop that debt from growing past the point of usefulness.

**D — how fast the gap is changing.** Not where the error is — where it's headed:

```
derivative(t) = ( error(t) - error(t-1) ) / dt
D = Kd * derivative        (Kd = 0.5)
```

This is the anticipation term. If speed is closing in on the target fast, the derivative senses that and eases off before overshoot happens — a damper, smoothing out what would otherwise be an oscillation. Its one real weakness: it reacts to *noise*, not just signal. And this simulation deliberately injects small random noise into the RPM reading — so the derivative term is, in a small way, always slightly jumpy by design.

All three combine into one number:

```
throttle(t) = clip( Kp*error(t) + Ki*integral(t) + Kd*derivative(t),  0,  100 )
```

The clip at the end isn't optional — without it, the math could hand back a negative throttle, or something past 100%, and neither of those mean anything to a real motor.

The gains themselves — 2.0, 0.02, 0.5 — weren't derived from a formula. They were found the way most PID gains are found in practice: tried, watched, adjusted. Push `Kp` up and the car responds faster but oscillates more. Push `Ki` up and steady-state error vanishes faster, at the cost of more windup risk. Push `Kd` up and the ride smooths out, but it gets twitchier around sensor noise. And all of it is quietly tied to `dt` — the derivative term divides by it directly, so changing the simulation's timestep without retuning the gains would change how the whole controller behaves, not just how often it updates.

---

## 2. Throttle becomes motion

Throttle is a percentage. A car doesn't move on percentages. So the first job is turning it into something physical.

```
motor_torque = max_torque * (throttle / 100)
```

That torque passes through the gearbox — multiplied up, the way gears do:

```
wheel_torque = motor_torque * gear_ratio
```

And torque at a wheel of a given radius becomes force at the point where rubber actually meets road:

```
motor_force = wheel_torque / wheel_radius
```

This is the number the whole rest of the simulation has been waiting for. This is the push.

---

## 3. Everything that pushes back

Nothing moves for free. Three forces resist the car, all at once, all the time.

**Rolling resistance** — the small, constant tax of tyres deforming against the road:

```
rolling_resistance = Crr * mass * g * cos(theta)
```

**Drag** — air, refusing to get out of the way. This one grows with the *square* of speed, which is why it barely matters at 20 km/h and dominates everything at 150. Double the speed and you don't double the drag — you quadruple it. That's the whole reason highway driving eats range so much faster than city driving does; the resistance isn't scaling with you, it's outrunning you.

```
drag_force = 0.5 * air_density * Cd * frontal_area * (speed / 3.6)^2
```

**Grade** — gravity, pulling the car back down whatever hill it's trying to climb:

```
grade_force = mass * g * sin(theta)
```

This one is small-sounding until you actually run the number. At a 4.5° incline — nothing dramatic, barely noticeable if you were walking it — `sin(4.5°) ≈ 0.078`. Multiply that through a 1500 kg car and it's adding roughly **1147 N** of resistance the motor has to fight, continuously, for as long as the climb lasts. That's not a rounding error. That's a real fraction of what the motor can put out at max torque.

`theta` is the road's grade, converted from degrees into radians before it ever touches `sin()` or `cos()` — a small detail, easy to forget, and silently wrong if you do: numpy's trig functions assume radians, not degrees, and there's no warning if you feed them the wrong unit. The road itself isn't a single number, either — it's a profile that changes as the drive goes on: flat, then a 4.5° climb, then a 2.0° descent, then a smaller 1.5° rise, then flat again. A journey, not a straight line.

Notice rolling resistance and grade force lean on the same angle in opposite ways — rolling resistance uses `cos(theta)` because only the part of the car's weight pressing straight down into the road creates friction, and that shrinks a little on a slope. Grade force uses `sin(theta)` because it's the part of gravity running *along* the slope, pulling the car back down it, that matters here. Same hill, two different components of the same force, doing two different jobs.

---

## 4. Giving a little back

When the driver lifts off the throttle and the car is still moving — the motor switches roles. Instead of pulling energy from the battery, it starts pushing a little back in.

```
if throttle < regen_throttle_threshold and speed > regen_speed_threshold:
    regen_current = max( -regen_gain * speed,  -regen_current_limit )
    motor_force  -= regen_brake_force
```

The current goes negative — flowing into the battery instead of out. The force goes down — the car resists its own momentum instead of adding to it. A small kindness, built into the coasting.

---

## 5. Newton doesn't care about any of this being a simulation

Once every force is known, the rest is just Newton's second law, done honestly.

```
net_force = motor_force - rolling_resistance - drag_force - grade_force
acceleration = net_force / mass
```

Speed updates, one small step at a time — never negative, because a simulated car, like a real one, doesn't reverse through zero by accident:

```
speed_ms(t) = max( speed_ms(t-1) + acceleration * dt,  0 )
speed_kmh   = speed_ms * 3.6
```

And distance just keeps counting, quietly, in the background:

```
distance_km += speed_kmh * dt / 3600
```

---

## 6. How hard the motor is actually working

Speed tells you RPM:

```
wheel_rpm = ( speed_ms / (2*pi*wheel_radius) ) * 60
rpm = max( wheel_rpm * gear_ratio + noise,  0 )
```

But RPM alone doesn't tell you efficiency. Real motors have a sweet spot — a place where they run cleanest — and get worse the further you push them from it, in either direction:

```
rpm_factor    = max( 0.5, 1 - ((rpm - peak_rpm) / 8000)^2 )
torque_factor = max( 0.6, 1 - ((motor_torque - 150) / 350)^2 )
motor_eff     = clip( base_motor_efficiency * rpm_factor * torque_factor,  0.70,  0.94 )
```

Not a perfect model of a real motor curve. Close enough to matter.

---

## 7. What all of this costs the battery

Mechanical power, first — torque doing its work at a given angular speed:

```
angular_velocity = rpm * 2*pi / 60
p_motor_kW       = motor_torque * angular_velocity / 1000
```

Then the honest part — converting that into what the battery actually feels. Driving costs more than the motor alone suggests, because nothing is perfectly efficient. Regen gives back less than it captures, for the same reason.

```
if p_motor_kW > 0:   # driving: losses stack, so more is drawn than the motor alone would suggest
    power = (p_motor_kW / motor_eff) / battery_efficiency
else:                 # regen: losses stack the other way, so less comes back than was captured
    power = p_motor_kW * motor_eff * battery_efficiency
```

---

## 8. The battery, paying attention to itself

Current isn't just the load from driving. There's always a baseline hum — electronics that never sleep:

```
current = quiescent_current + (throttle*0.8 + speed*0.15) + regen_current + noise
```

Voltage rises with how full the battery is:

```
ocv = 320 + 90 * (soc / 100)
```

But what you actually measure at the terminals is a little lower — internal resistance takes its cut:

```
voltage = ocv - current * battery_resistance
```

And charge just drains, tick by tick, however much power was spent:

```
battery_energy = clip( battery_energy - power*dt/3600,  0,  battery_capacity_kWh )
soc = (battery_energy / battery_capacity_kWh) * 100
```

---

## 9. Heat doesn't ask permission

Current through resistance makes heat. Always has. This is just Ohm's law refusing to be ignored:

```
heat_generation = current^2 * battery_resistance
```

The pack warms from that heat, and cools back toward the world around it, slowly, the way anything with mass does:

```
temp += heat_generation * thermal_mass_factor
temp -= (temp - ambient_temp) * cooling_rate
temp  = max(temp, ambient_temp)
```

`thermal_mass_factor` decides how quickly a spike in current shows up as a spike in temperature. `cooling_rate` decides how quickly the pack lets that heat go, once the current backs off. Together, they're the difference between a battery that reacts instantly and one that just — absorbs it, calmly, and evens out over time.

---

## 10. One number that isn't fed back into anything

Kinetic energy is logged, not used. Just there, quietly, as a way of checking the simulation's honesty against itself:

```
kinetic_energy_J = 0.5 * mass * speed_ms^2
```

---

## The whole tick, start to finish

```
target speed → gap → throttle
     → torque → force at the wheels
     → drag, rolling resistance, grade, pushing back
     → regen, if coasting
     → net force → acceleration → speed → distance
     → RPM → motor efficiency
     → power the battery actually pays
     → current, voltage, charge
     → heat → temperature
```

Then it writes the row, and does it again. A thousand times, usually, one second apart — the same handful of equations, over and over, quietly adding up to something that looks, from a distance, like driving.
