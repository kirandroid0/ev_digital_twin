# EV Digital Twin Simulator

Simulates an electric vehicle over a drive cycle — PID speed control, vehicle dynamics, battery/thermal model — and streams telemetry into PostgreSQL for live viewing in Grafana.

PLEASE READ  - [`docs/PHYSICS.md`](docs/PHYSICS.md) !!!!!!!!


<img width="1920" height="995" alt="image" src="https://github.com/user-attachments/assets/bd0f1a2a-2633-44ce-bea4-25fe704d63b9" />

<img width="1900" height="985" alt="image" src="https://github.com/user-attachments/assets/001cfee4-eab0-441e-8dda-1e9eab619562" />



## How it works

Each tick:
1. PID controller compares target speed to current speed, outputs throttle
2. Throttle → motor torque → wheel force
3. Subtract drag, rolling resistance, grade force
4. Update speed, distance
5. Compute motor efficiency, power draw
6. Update battery SoC, voltage, current, temperature
7. Write row to `telemetry`, repeat

```
PID controller → vehicle dynamics → battery/thermal model → PostgreSQL → Grafana
```


## Setup

```bash
pip install -r requirements.txt
cp .env.example .env   # fill in your DB credentials
python runner.py
```

Needs a running PostgreSQL instance with the database already created. The `telemetry` table is dropped and recreated every time.

## Config

Everything's in the `SimConfig` dataclass at the top of `runner.py` — mass, drag coefficient, battery capacity, PID gains, regen thresholds, `samples`/`dt` for run length and speed.

## Grafana

Point a PostgreSQL data source at the same DB, query `telemetry`. Only a tick counter (`time_s`) is stored, so map it to a real timestamp:

```sql
SELECT
  TO_TIMESTAMP(EXTRACT(EPOCH FROM NOW()) - ((SELECT MAX(time_s) FROM telemetry) - time_s)) AS "time",
  speed_kmh, target_speed_kmh
FROM telemetry
ORDER BY time_s;
```

## Limitations

Drive cycle steps hard instead of ramping. Battery model has no aging/degradation, no cell-level detail. Built to learn control systems and real-time data pipelines, not a validated vehicle model.
