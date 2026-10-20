# Actuator database

Motors, servos and motor controllers for all of Fernando's robotics projects, as plain CSV you can edit in Excel or a text editor. It lives in `omni-robot/database/` for now and will move one level up (`Robotics/database/`) so other projects can share it. Nothing in it depends on omni-robot.

| File | What it holds |
| --- | --- |
| `motors.csv` | Bare motors, gearmotors and integrated actuators. One row per part: the same motor with another gearbox is another row |
| `servos.csv` | Smart (serial-bus) and hobby (PWM) servos |
| `controllers.csv` | H-bridges, ESCs, FOC drivers and controllers, smart brushed controllers, MCUs used as motor hosts, and the DC-DC converters a function needs |
| `sources.csv` | Every source: id, kind, URL, what it supports, date checked |
| `actuator_db.py` | Schema, loader, validator, derived quantities, requirement checks, costs, compatibility, export to a sim catalog, gaps report |
| `test_actuator_db.py` | Its checks: `python -m pytest -q` in this folder |
| `gaps.csv` | Generated: the empty key fields per part (`python actuator_db.py gaps --csv`) |

Commands (use the sim venv, `C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe`):

```
python actuator_db.py validate        # run after every edit: schema, units, provenance, sources, consistency
python actuator_db.py gaps --csv      # what to look up or measure next (price first: cost is a primary criterion)
python actuator_db.py show dji-c610   # one part with its provenance and derived model
python actuator_db.py schema          # regenerates the schema tables below
```

## Rules

1. **SI units, written into the column name**: `torque_stall_Nm`, `speed_noload_rpm`, `mass_kg`, `price_usd`. rpm is the one non-SI unit kept (for speeds); convert kg cm to N m (× 0.0980665).
2. **Output shaft vs motor side.** Speeds and torques (`speed_*`, `torque_*`) are at the output, after any gearbox. `kv_rpm_per_V`, `kt_Nm_per_A`, `ke_Vs_per_rad`, `r_ohm` and `rotor_inertia_kgm2` are motor side. `r_kind` says whether `r_ohm` is per phase, line-to-line or terminal.
3. **Blank means unknown. Never invent a number.** An estimate is allowed only when it is marked `est` in the provenance columns and explained in `notes`.
4. **Provenance per value.** `prov_default` (datasheet / reseller / forum / est / measured) covers every filled value in the row; `prov_overrides` lists the exceptions, e.g. `est: rotor_inertia_kgm2, gear_eff; reseller: price_usd`. A reseller's number is never written as a datasheet value: a reseller page quoting a spec is `reseller`, even when it probably copied the maker.
5. **Sources and dates.** `source_ids` lists the `sources.csv` rows behind the row; `date_checked` is when the values and price were last checked (YYYY-MM-DD). Re-check prices before ordering.
6. **Prices** are USD at quantity 1 from `vendor`; quantity breaks go in `price_breaks` (`5:56.07;25:51.59`). A converted price (EUR, GBP, INR, AUD) is `est`, with the original in `notes`.
7. **Bench data goes in the `meas_*` columns** (motors and servos), with `meas_date` and `meas_notes`. Where a measured value exists, `actuator_db` uses it instead of the datasheet value (`Part.meas`, `dc_model(measured=True)`). Don't overwrite the datasheet columns: the difference between the two is the sim-to-real gap.
8. **ids are permanent** (projects refer to them): lowercase `maker-part-variant`, letters, digits, `-` and `.`.

## Adding a part

1. Add the source(s) to `sources.csv` (next free `S0nn`, kind maker / reseller / forum / project).
2. Add the row to the right CSV. Fill what the source states, in SI, at the output shaft; leave the rest blank. For a gearmotor, the five points (`v_test_V`, `speed_noload_rpm`, `current_noload_A`, `torque_stall_Nm`, `current_stall_A`) are what the DC model needs; set `stall_extrapolated` when the maker says so.
3. Set `prov_default`, `prov_overrides`, `source_ids`, `date_checked`, and say in `notes` what is uncertain (VERIFY ...).
4. Controllers: set `ok_7s_full` from `v_max_V` (yes ≥ 31.4 V, marginal 29.4-31.4 V, no below, VERIFY when the maximum isn't published). `validate` checks it.
5. Run `python actuator_db.py validate`, then `gaps --csv`.

## What the loader does

- **Derived model** (`dc_model`): R, kt, ke and drag at the output from the five datasheet points (as omni-robot's `catalog.FeedMotor`), or from kt / kv / R / gear when the points are missing. Then: torque available at a speed and supply with a current limit (`torque_at`), no-load speed at any voltage, copper loss at an RMS torque, output speed at a wheel diameter (`output_speed_ms`, `wheel_rpm`). Servos: a linear curve from stall torque and no-load speed.
- **Requirement checks** (`Requirement`, `check`, `passes`): speed at the minimum supply, torque left at speed, peak torque (part and controller current limit), RMS torque against a steady-state thermal model (when `r_th_KperW` and `t_winding_max_C` are known) or the rated torque, holding torque, surviving the maximum supply (part and controller), interface, integration level, feedback, mass, price. Each check is PASS / FAIL / UNKNOWN; an unknown is a gap, not a pass.
- **Figures of merit**: `function_cost` (part + its share of a controller board + power conversion), `usd_per_Nm` at the operating speed and supply, `Nm_per_kg`.
- **Compatibility** (`compatible`): motor type vs controller, the supply against both maximums (unknown maximums are flagged), current, encoder inputs for a controller's loops, and power-stage-only FOC drivers (DIY).
- **Export to a sim catalog**: `to_drive_motor`, `to_feed_motor`, `to_flywheel_motor`, `to_esc` build omni-robot's `catalog.py` dataclasses (`sim_catalog(path)` imports the project's catalog). `test_export_matches_the_sim_catalog` checks the round trip.
- **Gaps** (`gaps`, `gaps --csv`): blank key fields per part (key columns are marked * below), tuned per motor kind.

## Schema

Columns marked * are key fields: the gaps report lists them when blank.

### motors.csv

| Column | Type | Meaning |
| --- | --- | --- |
| `id` | str | Unique slug, maker-part (e.g. pololu-37d-24v-19-enc). Never reuse or rename: projects refer to it |
| `manufacturer` | str | Maker |
| `part_number` | str | Maker's part number or SKU |
| `name` | str | Descriptive name |
| `vendor` | str | Where the price comes from (shop) |
| `kind` * | enum (brushed_dc / bldc_outrunner / bldc_inrunner / gimbal / stepper / closed_loop_stepper / hub_motor / integrated_actuator) | Motor class |
| `integration` * | enum (bare / gearbox / encoder / gearbox_encoder / driver / smart) | What comes in the box: bare motor / +gearbox / +encoder / +gearbox+encoder / +driver (no loops) / smart (driver with its own closed loops) |
| `commutation` * | enum (brushed / sensorless / hall / encoder_foc / stepper_open / stepper_closed) | brushed, sensorless BLDC, Hall BLDC, encoder FOC, open- or closed-loop stepping |
| `sensor_type` | str | Built-in sensor: hall, magnetic, optical, none |
| `sensor_absolute` | enum (yes / no / partly / VERIFY) | Absolute (yes) or incremental (no) position |
| `sensor_cpr_motor` | float | Counts per motor revolution (quadrature counts where it applies) |
| `sensor_cpr_output` * | float | Counts per output revolution |
| `sensor_multiturn` | enum (yes / no / partly / VERIFY) | Keeps the turn count over power cycles |
| `gear_type` | enum (none / spur / planetary / worm / harmonic / cycloidal / belt / spur_helical) | Reduction |
| `gear_ratio` * | float | Reduction ratio (1 = direct drive) |
| `gear_eff` | float | Gearbox efficiency 0-1 |
| `backlash_arcmin` | float | Output backlash, arc minutes |
| `backdrivable` | enum (yes / no / partly / VERIFY) | Can a load turn the unpowered output |
| `v_rated_V` * | float | Rated supply voltage |
| `v_min_V` | float | Lowest allowed supply voltage |
| `v_max_V` * | float | Highest allowed supply voltage |
| `kv_rpm_per_V` | float | Speed constant, motor side |
| `kt_Nm_per_A` | float | Torque constant, motor side (per phase amp / q-axis amp for BLDC) |
| `ke_Vs_per_rad` | float | Back-EMF constant, motor side |
| `r_ohm` | float | Winding resistance (see r_kind) |
| `r_kind` | enum (phase / line_line / terminal / unclear) | How r_ohm is measured |
| `inductance_H` | float | Winding inductance (same convention as r_kind) |
| `pole_pairs` | int | Rotor pole pairs |
| `v_test_V` * | float | Voltage of the no-load / stall / rated points below |
| `speed_noload_rpm` * | float | Output no-load speed at v_test_V |
| `current_noload_A` | float | No-load current at v_test_V |
| `torque_stall_Nm` | float | Output stall torque at v_test_V |
| `current_stall_A` | float | Stall current at v_test_V |
| `stall_extrapolated` | enum (yes / no / partly / VERIFY) | The maker extrapolated the stall point (common for gearmotors) |
| `speed_rated_rpm` | float | Output speed at the rated point |
| `torque_rated_Nm` * | float | Output continuous (rated) torque |
| `current_rated_A` | float | Current at the rated point |
| `torque_peak_Nm` * | float | Output peak torque (driver or gear limit) |
| `current_peak_A` | float | Peak current |
| `rotor_inertia_kgm2` | float | Rotor inertia, motor side |
| `mass_kg` * | float | Mass as sold (with encoder / driver if integrated) |
| `diameter_m` | float | Body diameter (or width for square frames) |
| `length_m` | float | Body length without shaft |
| `shaft` | str | Output shaft or flange (e.g. 6 mm D, 16 mm long) |
| `mounting` | str | Mounting pattern (e.g. 6x M3 on PCD 31) |
| `t_winding_max_C` | float | Maximum winding temperature |
| `r_th_KperW` | float | Winding-to-ambient thermal resistance (steady state) |
| `thermal_notes` | str | Duty limits, derating, time constants |
| `brush_life_h` | float | Brush life, hours (brushed only) |
| `noise` | str | Noise notes or dB |
| `ip_rating` | str | Ingress protection |
| `interface` * | list | none, pwm, dshot, can, can_fd, rs485, ttl_bus, uart, usb, i2c, step_dir, quadrature |
| `control_modes` | list | torque, speed, position, mit (impedance), voltage |
| `telemetry` | list | position, speed, current, torque, temperature, voltage, errors |
| `protections` | list | over_voltage, under_voltage, over_current, thermal, stall, comm_loss |
| `controller` | str | Controller it needs: 'integrated', a controllers.csv id, or blank (any of its type) |
| `price_usd` * | float | Unit price in USD at quantity 1 (converted prices are est) |
| `price_breaks` | str | Quantity breaks 'qty:usd;qty:usd' (e.g. 5:56.07;25:51.59) |
| `availability` | str | Stock seen on date_checked (in stock / out of stock / backorder / discontinued) |
| `prov_default` | enum (datasheet / reseller / forum / est / measured) | Provenance of every filled value unless overridden |
| `prov_overrides` | str | Exceptions: 'est: col, col; reseller: col' (column names of this table) |
| `source_ids` | list | Source ids from sources.csv, ';'-separated |
| `date_checked` | date | YYYY-MM-DD the values and price were last checked |
| `notes` | str | Anything else: caveats, VERIFY items, why rejected |
| `meas_speed_noload_rpm` | float | MEASURED output no-load speed at meas_v_V |
| `meas_current_noload_A` | float | MEASURED no-load current |
| `meas_torque_stall_Nm` | float | MEASURED stall torque (or locked-rotor torque at a stated current) |
| `meas_current_stall_A` | float | MEASURED stall current |
| `meas_v_V` | float | Voltage of the measured points |
| `meas_r_ohm` | float | MEASURED resistance (r_kind convention) |
| `meas_kt_Nm_per_A` | float | MEASURED torque constant, motor side |
| `meas_rotor_inertia_kgm2` | float | MEASURED rotor inertia |
| `meas_backlash_arcmin` | float | MEASURED backlash |
| `meas_r_th_KperW` | float | MEASURED winding-to-ambient thermal resistance |
| `meas_mass_kg` | float | MEASURED mass |
| `meas_date` | date | Date of the bench test |
| `meas_notes` | str | Bench setup, log file |

### servos.csv

| Column | Type | Meaning |
| --- | --- | --- |
| `id` | str | Unique slug, maker-part (e.g. pololu-37d-24v-19-enc). Never reuse or rename: projects refer to it |
| `manufacturer` | str | Maker |
| `part_number` | str | Maker's part number or SKU |
| `name` | str | Descriptive name |
| `vendor` | str | Where the price comes from (shop) |
| `kind` * | enum (smart_bus / hobby_pwm / industrial) | smart_bus (serial bus with feedback), hobby_pwm (no feedback), industrial |
| `v_test_V` * | float | Voltage of the torque / speed figures |
| `v_min_V` | float | Lowest supply voltage |
| `v_max_V` * | float | Highest supply voltage |
| `torque_stall_Nm` * | float | Stall torque at v_test_V |
| `torque_rated_Nm` | float | Rated (continuous) torque |
| `speed_s_per_60deg` * | float | No-load speed, seconds per 60 degrees, at v_test_V |
| `speed_noload_rad_s` | float | Same speed in rad/s (derived when blank: (pi/3) / s_per_60deg) |
| `current_stall_A` | float | Stall current |
| `current_noload_A` | float | No-load current |
| `gear_ratio` | float | Gear ratio |
| `gear_material` | str | metal, steel, plastic ... |
| `feedback` * | list | position, speed, load, current, temperature, voltage (blank list = none) |
| `position_bits` | float | Position resolution, bits per turn |
| `absolute` | enum (yes / no / partly / VERIFY) | Absolute position over power cycles |
| `range_deg` | float | Servo travel in position mode |
| `continuous_rotation` | enum (yes / no / partly / VERIFY) | Has a wheel / speed mode |
| `bus` * | str | pwm, ttl_half_duplex, rs485, can ... |
| `baud_max` | float | Highest bus rate, bit/s |
| `mass_kg` * | float | Mass |
| `size_mm` | str | L x W x H |
| `ip_rating` | str | Ingress protection |
| `price_usd` * | float | Unit price in USD at quantity 1 (converted prices are est) |
| `price_breaks` | str | Quantity breaks 'qty:usd;qty:usd' (e.g. 5:56.07;25:51.59) |
| `availability` | str | Stock seen on date_checked (in stock / out of stock / backorder / discontinued) |
| `prov_default` | enum (datasheet / reseller / forum / est / measured) | Provenance of every filled value unless overridden |
| `prov_overrides` | str | Exceptions: 'est: col, col; reseller: col' (column names of this table) |
| `source_ids` | list | Source ids from sources.csv, ';'-separated |
| `date_checked` | date | YYYY-MM-DD the values and price were last checked |
| `notes` | str | Anything else: caveats, VERIFY items, why rejected |
| `meas_torque_stall_Nm` | float | MEASURED stall torque |
| `meas_speed_noload_rad_s` | float | MEASURED no-load speed |
| `meas_v_V` | float | Voltage of the measurements |
| `meas_date` | date | Date of the bench test |
| `meas_notes` | str | Bench setup, log file |

### controllers.csv

| Column | Type | Meaning |
| --- | --- | --- |
| `id` | str | Unique slug, maker-part (e.g. pololu-37d-24v-19-enc). Never reuse or rename: projects refer to it |
| `manufacturer` | str | Maker |
| `part_number` | str | Maker's part number or SKU |
| `name` | str | Descriptive name |
| `vendor` | str | Where the price comes from (shop) |
| `kind` * | enum (h_bridge / esc / foc_driver / foc_controller / smart_controller / stepper_driver / mcu / dcdc) | h_bridge (brushed), esc (BLDC, RC style), foc_driver (power stage only), foc_controller (runs FOC + loops), smart_controller (brushed with its own loops), stepper_driver, mcu, dcdc |
| `channels` * | int | Motors per board |
| `motor_types` * | list | brushed, bldc, stepper |
| `power_stage` | str | full_h_bridge, half_bridge, 3_phase, none |
| `v_min_V` * | float | Lowest supply voltage |
| `v_max_V` * | float | Highest supply voltage |
| `ok_7s_full` * | enum (yes / marginal / no / VERIFY) | Takes a full 7S pack (29.4 V): yes (>= 31.4 V), marginal (29.4-31.4), no, VERIFY (unknown max). Checked against v_max_V by validate |
| `i_cont_A` * | float | Continuous current per channel |
| `i_peak_A` | float | Peak current per channel |
| `current_sense` | str | How current is sensed / reported |
| `current_regulation` | str | Current limiting (cycle-by-cycle, FOC torque loop, none) |
| `encoder_inputs` | int | Encoder inputs (total) |
| `encoder_type` | str | quadrature, hall, spi_absolute, motor's own ... |
| `encoder_max_rate_Hz` | float | Highest count rate |
| `loops` * | list | none, current, speed_pid, position_pid, foc, autotune |
| `interface` * | list | pwm, dshot, can, can_fd, uart, packet_serial, usb, i2c, rc_pwm, analog, step_dir |
| `logic_level_V` | str | Logic levels |
| `telemetry` | list | speed, position, current, voltage, temperature, errors |
| `protections` | list | over_voltage, under_voltage, over_current, thermal, reverse_polarity, comm_loss |
| `regen` | str | Regenerative braking handling |
| `size_mm` | str | L x W (x H) |
| `mass_kg` | float | Mass |
| `config_tool` | str | Configuration software |
| `price_usd` * | float | Unit price in USD at quantity 1 (converted prices are est) |
| `price_breaks` | str | Quantity breaks 'qty:usd;qty:usd' (e.g. 5:56.07;25:51.59) |
| `availability` | str | Stock seen on date_checked (in stock / out of stock / backorder / discontinued) |
| `prov_default` | enum (datasheet / reseller / forum / est / measured) | Provenance of every filled value unless overridden |
| `prov_overrides` | str | Exceptions: 'est: col, col; reseller: col' (column names of this table) |
| `source_ids` | list | Source ids from sources.csv, ';'-separated |
| `date_checked` | date | YYYY-MM-DD the values and price were last checked |
| `notes` | str | Anything else: caveats, VERIFY items, why rejected |

### sources.csv

| Column | Type | Meaning |
| --- | --- | --- |
| `id` | str | Source id (S001 ...) |
| `kind` | enum (maker / reseller / forum / project) | maker (maker's page or datasheet), reseller, forum, project (own docs / purchase records) |
| `url` | str | Link (or a repo path for project sources) |
| `supports` | str | Which parts and values it supports |
| `date_checked` | date | YYYY-MM-DD |
| `notes` | str | Notes |

