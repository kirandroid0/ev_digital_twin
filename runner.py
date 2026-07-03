from dataclasses import dataclass
import numpy as np
import pandas as pd
from sqlalchemy import create_engine, text
import time


@dataclass
class SimConfig:
    # Connection
    db_password: str = "12345"
    db_host: str = "localhost"
    db_port: int = 5432
    db_name: str = "evdb"

    # Timing
    samples: int = 1000
    dt: float = 1.0           # seconds per tick
    batch_size: int = 10      # rows buffered before each DB write

    # Vehicle
    vehicle_mass: float = 1500.0   # kg
    wheel_radius: float = 0.30     # m
    gear_ratio: float = 9.0        # motor shaft : wheel shaft

    # Aero / road
    air_density: float = 1.225     # kg/m³
    Cd: float = 0.28               # drag coefficient
    frontal_area: float = 2.2      # m²
    Crr: float = 0.01              # rolling resistance coeff
    g: float = 9.81                # m/s²

    # Battery
    battery_capacity_kWh: float = 60.0
    battery_resistance: float = 0.05   # Ω internal resistance
    battery_efficiency: float = 0.92

    # Motor
    base_motor_efficiency: float = 0.90
    max_torque: float = 350.0          # Nm
    peak_rpm: float = 4000.0           # RPM at peak efficiency

    # Thermal
    ambient_temp: float = 25.0         # °C
    thermal_mass_factor: float = 0.00002
    cooling_rate: float = 0.002

    # PID
    Kp: float = 2.0
    Ki: float = 0.02
    Kd: float = 0.5
    integral_max: float = 500.0        # anti-windup clamp 

    # Regen
    regen_speed_threshold: float = 10.0   # km/h — below this, no regen
    regen_throttle_threshold: float = 5.0 # % throttle — above this, no regen
    regen_gain: float = 0.15
    regen_current_limit: float = 40.0     # A max charge current
    regen_brake_force: float = 400.0      # N retarding force


cfg = SimConfig()


#database setup 

engine = create_engine(
    f"postgresql+psycopg2://postgres:{cfg.db_password}"
    f"@{cfg.db_host}:{cfg.db_port}/{cfg.db_name}"
)

with engine.begin() as conn:
    conn.execute(text("DROP TABLE IF EXISTS telemetry CASCADE"))
    print("Old table dropped (or did not exist).")


#drive cycle

def build_drive_cycle(samples: int) -> list[float]:
    """Target speed (km/h) for each second."""
    out = []
    for t in range(samples):
        if   t < 100:  out.append(0.0)
        elif t < 250:  out.append(30.0)
        elif t < 400:  out.append(50.0)
        elif t < 500:  out.append(0.0)
        elif t < 700:  out.append(80.0)
        elif t < 850:  out.append(40.0)
        else:          out.append(0.0)
    return out

def build_road_profile(samples: int) -> list[float]:
    """Road grade (degrees) for each second."""
    out = []
    for t in range(samples):
        if   t < 200:  out.append(0.0)
        elif t < 450:  out.append(4.5)
        elif t < 700:  out.append(-2.0)
        elif t < 850:  out.append(1.5)
        else:          out.append(0.0)
    return out

target_speed  = build_drive_cycle(cfg.samples)
road_profile  = build_road_profile(cfg.samples)


#initialization

speed            = 0.0
temp             = cfg.ambient_temp + 5.0
soc              = 100.0
distance         = 0.0
battery_energy   = cfg.battery_capacity_kWh

# PID state
integral         = 0.0
previous_error   = 0.0


#MAIN SIM LOOP

batch: list[dict] = []

with engine.connect() as conn:
    print(f"Simulation started — {cfg.samples} ticks at {cfg.dt}s each.")

    for t in range(cfg.samples):

        # ── PID CONTROLLER ─────────────────────────────────────────
        error       = target_speed[t] - speed
        integral   += error * cfg.dt

       
        integral    = np.clip(integral, -cfg.integral_max, cfg.integral_max)

        derivative  = (error - previous_error) / cfg.dt
        throttle    = np.clip(
            cfg.Kp * error + cfg.Ki * integral + cfg.Kd * derivative,
            0.0, 100.0
        )
        previous_error = error

    
        motor_torque  = cfg.max_torque * throttle / 100.0          # Nm at motor shaft
        wheel_torque  = motor_torque * cfg.gear_ratio               # Nm at wheel (via gearbox)
        motor_force   = wheel_torque / cfg.wheel_radius             # N at tyre contact patch

        road_grade_deg = road_profile[t]
        theta_rad      = np.radians(road_grade_deg)

        rolling_resistance = cfg.Crr * cfg.vehicle_mass * cfg.g * np.cos(theta_rad)
        drag_force         = 0.5 * cfg.air_density * cfg.Cd * cfg.frontal_area * (speed / 3.6) ** 2
        grade_force        = cfg.vehicle_mass * cfg.g * np.sin(theta_rad)

       
        regen_current = 0.0

        if throttle < cfg.regen_throttle_threshold and speed > cfg.regen_speed_threshold:
            regen_current = max(-cfg.regen_gain * speed, -cfg.regen_current_limit)
            motor_force  -= cfg.regen_brake_force

        # ── VEHICLE KINEMATICS ─────────────────────────────────────
        net_force    = motor_force - rolling_resistance - drag_force - grade_force
        acceleration = net_force / cfg.vehicle_mass

        speed_ms     = max(speed / 3.6 + acceleration * cfg.dt, 0.0)
        speed        = speed_ms * 3.6
        distance    += speed * cfg.dt / 3600.0

        # ── RPM & MOTOR EFFICIENCY ─────────────────────────────────
        wheel_rpm = (speed_ms / (2.0 * np.pi * cfg.wheel_radius)) * 60.0
        rpm       = max(wheel_rpm * cfg.gear_ratio + np.random.normal(0, 20), 0.0)

        rpm_factor    = max(0.5, 1.0 - ((rpm    - cfg.peak_rpm)  / 8000.0) ** 2)
        torque_factor = max(0.6, 1.0 - ((motor_torque - 150.0)   / 350.0)  ** 2)
        motor_eff     = float(np.clip(
            cfg.base_motor_efficiency * rpm_factor * torque_factor, 0.70, 0.94
        ))

        # ── POWER BALANCE ──────────────────────────────────────────
        angular_velocity = rpm * 2.0 * np.pi / 60.0
        p_motor_kW       = motor_torque * angular_velocity / 1000.0

        if p_motor_kW > 0:
            power = (p_motor_kW / motor_eff) / cfg.battery_efficiency   # draw from pack
        else:
            power = p_motor_kW * motor_eff * cfg.battery_efficiency      # coast / regen

        # ── BATTERY ELECTRICAL MODEL ───────────────────────────────
        quiescent_current = 10.0                          # A — always-on systems
        load_current      = throttle * 0.8 + speed * 0.15
        current = quiescent_current + load_current + regen_current + np.random.normal(0, 2)
        current = max(current, -cfg.regen_current_limit)

        ocv     = 320.0 + 90.0 * (soc / 100.0)
        voltage = ocv - current * cfg.battery_resistance

        # Energy integration — power is negative during regen so subtraction
        battery_energy = float(np.clip(
            battery_energy - (power * cfg.dt / 3600.0),
            0.0,
            cfg.battery_capacity_kWh
        ))
        soc = (battery_energy / cfg.battery_capacity_kWh) * 100.0

        # ── THERMAL MODEL ──────────────────────────────────────────
        heat_generation = current ** 2 * cfg.battery_resistance
        temp += heat_generation * cfg.thermal_mass_factor
        temp -= (temp - cfg.ambient_temp) * cfg.cooling_rate
        temp  = max(temp, cfg.ambient_temp)

        # ── KINETIC ENERGY ─────────────────
        kinetic_energy_J = 0.5 * cfg.vehicle_mass * speed_ms ** 2

        # ──assemble
        batch.append({
            "time_s":               t,
            "target_speed_kmh":     round(target_speed[t],       1),
            "speed_kmh":            round(speed,                  1),
            "distance_km":          round(distance,               3),
            "throttle_pct":         round(float(throttle),        1),
            "acceleration_ms2":     round(acceleration,           3),
            "motor_torque_Nm":      round(motor_torque,           1),
            "wheel_torque_Nm":      round(wheel_torque,           1), 
            "motor_force_N":        round(motor_force,            1),
            "drag_force_N":         round(drag_force,             1),
            "rolling_resistance_N": round(rolling_resistance,     1),
            "grade_force_N":        round(grade_force,            1),   
            "net_force_N":          round(net_force,              1),  
            "regen_current_A":      round(regen_current,          1),
            "rpm":                  round(rpm,                    0),
            "battery_current_A":    round(current,                1),
            "battery_voltage_V":    round(voltage,                1),
            "battery_power_kW":     round(power,                  2),
            "battery_heat_W":       round(heat_generation,        2),
            "battery_temp_C":       round(temp,                   2),
            "soc_pct":              round(soc,                    2),
            "road_grade":           round(road_grade_deg,         1),
            "motor_efficiency_pct": round(motor_eff * 100.0,      1),
            "kinetic_energy_J":     round(kinetic_energy_J,       0),  
        })

       
        if len(batch) >= cfg.batch_size:
            pd.DataFrame(batch).to_sql(
                "telemetry", conn, if_exists="append", index=False
            )
            conn.commit()   # explicit commit — don't rely on implicit behaviour
            batch.clear()
            print(f"  t={t:4d} | speed={speed:5.1f} km/h | SoC={soc:5.1f}% | temp={temp:4.1f}°C")

        time.sleep(cfg.dt)

    ###
    if batch:
        pd.DataFrame(batch).to_sql(
            "telemetry", conn, if_exists="append", index=False
        )
        conn.commit()

print("\nSimulation complete.")
