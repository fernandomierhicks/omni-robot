# Parts database: actuators, robot computers, cameras

Motors, servos, motor controllers, robot computers (SBCs, Jetson modules, mini PCs) and cameras for all of Fernando's robotics projects, as plain CSV you can edit in Excel or a text editor. It lives in `omni-robot/database/` for now and will move one level up (`Robotics/database/`) so other projects can share it. Nothing in it depends on omni-robot. (The module keeps its name, `actuator_db.py`, because projects import it.)

| File | What it holds |
| --- | --- |
| `motors.csv` | Bare motors, gearmotors and integrated actuators. One row per part: the same motor with another gearbox is another row |
| `servos.csv` | Smart (serial-bus) and hobby (PWM) servos |
| `controllers.csv` | H-bridges, ESCs, FOC drivers and controllers, smart brushed controllers, MCUs used as motor hosts, and the DC-DC converters a function needs |
| `motor_tests.csv` | Third-party bench tests (dynamometer runs): one row per test with its maxima, linked to `motors.csv` by `motor_id` |
| `computers.csv` | Robot computers: Jetson dev kits, modules and carriers, SBCs (Pi, RK3588, QCS6490, x86 boards), mini PCs, add-on NPUs. CPU benchmarks for scaling a measured software load, OS / ROS 2 support, camera connectors, input voltage, power, price |
| `cameras.csv` | Cameras for robots: MIPI modules, synced stereo kits, USB cameras, capture boards (OAK-FFC, GMSL cards), depth cameras. Shutter, colour, modes, hardware trigger and how a pair syncs, hosts, driver status, published latency, price |
| `sources.csv` | Every source: id, kind, URL, what it supports, date checked |
| `actuator_db.py` | Schema, loader, validator, derived quantities, requirement checks (actuators, computers, cameras), costs, compatibility, export to a sim catalog, gaps report |
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
4. **Provenance per value.** `prov_default` (datasheet / reseller / forum / est / measured / third_party / press: third_party for a third-party test database such as Tyto Robotics', Geekbench or sbc-bench; press for a news or review site reporting the maker's figures or prices) covers every filled value in the row; `prov_overrides` lists the exceptions, e.g. `est: rotor_inertia_kgm2, gear_eff; reseller: price_usd`. A reseller's number is never written as a datasheet value: a reseller page quoting a spec is `reseller`, even when it probably copied the maker.
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

## What is in it, and where it came from (October 2026)

- **Maker catalogs, complete:**
  - Pololu micro metal gearmotors (every class and voltage), 25D and 37D (every ratio and voltage);
  - the Dynamixel X series (one row per model; the other voltages are in its notes) plus prices for the AX / MX / P lines;
  - the CubeMars AK, AKA, AKH and AKE actuators, the GL gimbal motors and the RI / RO frameless motors;
  - ODrive's motors and controllers (S1, Pro, Micro, v3.6);
  - every RoboClaw, the SimpleFOC boards, SOLO MINI, VESC (Trampa, Makerbase, Flipsky);
  - Damiao, RobStride, MyActuator, Unitree, Xiaomi, DJI, LK-TECH, Feetech, Waveshare, Hiwonder, goBILDA, REV.
- **Tyto Robotics propulsion database** (`tyto-*` motors, all of `motor_tests.csv`): 121 drone motors and up to 8 dynamometer tests each (424 tests). The site embeds each test's data as JSON in the page, and its own `/tests/search` endpoint lists every test. Each row keeps the run's maxima (rpm, current, power, torque, thrust, motor + ESC efficiency) and its URL. Uploads are user-made: some tests link a motor that differs from the one in the title (flagged in `notes`).
- **RobotShop listings** (`rs-*` motors):
  - all 18 pages of the brushless DC gear motors collection;
  - the first 3 pages (best sellers) of the BLDC and planetary gearmotor collections;
  - all 6 pages of the smart servos.

  Specs are parsed from product titles only: voltage, rpm, KV, torque, frame size and ratio. Kind, commutation and gear type are inferred (marked `est`). RobotShop sits behind Cloudflare, so these were read page by page. The remaining collections (planetary pages 4-23, BLDC 4-17, brushed / spur / stepper / RC servo) are not imported yet.

### Pendulum actuator follow-up (November 1, 2026)

Six missing models discovered through Pendulum's comparison tool are now included: SteadyWin GIM4305-10, GIM4310-10 and GIM3510-8; RobStride 05 and EduLite 05; Westwood Koala BEAR KB02. GIM3510-8 has separate GDS34 and GDZ34-with-secondary-encoder rows because the supplier publishes different torque, speed and voltage limits for the two drivers (seven configurations in total). Sources S211-S220 record the comparison page, exact supplier parameter images, configured-price JSON, manufacturer manuals and Westwood's store page. These are catalog claims, not our bench measurements or adopted robot parts.

- **SteadyWin:** use each driver's own table and the price of that complete configuration. The headline prices can describe a bare motor or a USB adapter. Pendulum's torque/mass figures differ from the supplier tables; the conflicts and missing thermal conditions are recorded in each row. The tables label speed as maximum, without specifying load; `speed_noload_rpm` therefore carries `est` provenance. Output-side Kv/Kt are not entered into motor-side columns. Optional output-encoder mass/dimensions are unconfirmed.
- **RobStride:** the July 13, 2026 CAN 2.0 manuals specify no-load speed at **48 V**, and rated torque tested at 100 rpm with a 70 x 70 mm heat sink. Mechanical details and reference prices from Pendulum retain `reseller` provenance. Phase RMS/peak currents are not mixed with DC current figures. The separate CAN FD variant is not represented by these rows.
- **Koala BEAR:** use the air-cooled continuous rating; peak torque is limited to 1.5 seconds. The 24 V no-load speed is an estimate from the published output Kv, not a measured operating point. Rated speed, interface details and output-bearing loads remain gaps.

### Computers and cameras (October 28, 2026)

Read for omni-robot's computer and stereo-camera choice (its `simulation/v11 sil/studies/compute_choice.py`, design.md 4.9); kept general so other robots can use them.

- **Computers (24):** every Orin module and the Orin Nano Super kit, AGX Orin, Thor and the announced Orin Nano 2 (H1 2027); Raspberry Pi 5 2 / 4 / 8 / 16 GB and the AI HAT+ 2; Radxa ROCK 5B+ (RK3588); Thundercomm RUBIK Pi 3 (QCS6490); LattePanda IOTA and ODROID-H4 Ultra (x86 SBCs); N150, Core Ultra 5 125H, Ryzen 7 8845HS and Panther Lake mini PCs; GMKtec M8 (Ryzen 5 PRO 6650H, $389.99) and M6 Ultra (Ryzen 5 7640HS, $499.99), complete with RAM and SSD, from GMKtec's store on Oct 28.
- **What changed in 2026:** NVIDIA raised every Jetson price in July (Orin Nano Super kit $249 → $399, Orin NX 16GB $599 → $999 at 1KU); Raspberry Pi list prices rose again on Oct 1 (Pi 5 2GB $77.50, 4GB $110, 8GB $180, 16GB $305; Adafruit sells the 4GB at $130 and 8GB at $200); x86 boards suffer too (ODROID-H4 discontinued, LattePanda IOTA $339). Mini PCs with RAM included held up better: Shopify stores' `products.json` gives every variant's live price and stock. JetPack 7.2 (June 2026) put Ubuntu 24.04 on every Orin, so ROS 2 Jazzy installs natively.
- **CPU benchmarks** for scaling a load measured on another machine: Geekbench 6 single / multi and sbc-bench's 7-Zip MIPS (single and all threads). `cpu_ratio`, `cores_needed` and `check_compute` bracket a load between the two benchmarks; little cores (A55, E-cores) count `little_core_weight` (0.35, est) of a big one.
- **Cameras (17):** colour global-shutter options per platform. Jetson: e-con e-CAM25_CUONX (AR0234, the only one with a vendor-documented hardware trigger on Orin Nano / NX), Arducam's AR0234 synced stereo kit, Arducam's AR0234 xISP (no hardware trigger), Arducam's OV9782 quad kit (45 fps), Stereolabs' GMSL2 ZED X One GS + ZED Link. Raspberry Pi: the official Global Shutter camera (IMX296, documented XTR trigger), Arducam AR0234 / OV9281 kits. Any USB host: Luxonis OAK-FFC 4P + OV9782 modules (FSYNC < 10 us, 24.5 ms published time-to-host), Arducam's OV9782 UVC camera (MJPEG), Orbbec Gemini 335L, RealSense D455, OAK 4 D. `check_camera` and `camera_fits` check them against a requirement and a computer.
- **Watch out:** Arducam's and e-con's Jetson drivers are built per L4T release: check a JetPack 7.2 driver exists before buying. The single OV9782 MIPI module (Arducam B0296) is discontinued. The Pi Global Shutter camera ($50) is thin on stock in the US (Oct 28: Mouser listed 152; Adafruit and Micro Center out) and comes without a lens.

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
| `availability` | str | Stock seen on date_checked (in stock / out of stock / backorder / discontinued), or in hand (owned, Fernando's bench) |
| `prov_default` | enum (datasheet / reseller / forum / est / measured / third_party / press) | Provenance of every filled value unless overridden |
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
| `availability` | str | Stock seen on date_checked (in stock / out of stock / backorder / discontinued), or in hand (owned, Fernando's bench) |
| `prov_default` | enum (datasheet / reseller / forum / est / measured / third_party / press) | Provenance of every filled value unless overridden |
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
| `availability` | str | Stock seen on date_checked (in stock / out of stock / backorder / discontinued), or in hand (owned, Fernando's bench) |
| `prov_default` | enum (datasheet / reseller / forum / est / measured / third_party / press) | Provenance of every filled value unless overridden |
| `prov_overrides` | str | Exceptions: 'est: col, col; reseller: col' (column names of this table) |
| `source_ids` | list | Source ids from sources.csv, ';'-separated |
| `date_checked` | date | YYYY-MM-DD the values and price were last checked |
| `notes` | str | Anything else: caveats, VERIFY items, why rejected |

### motor_tests.csv

| Column | Type | Meaning |
| --- | --- | --- |
| `id` | str | Test id (e.g. tyto-7ee: source prefix + the database's hash) |
| `motor_id` | str | motors.csv id of the motor tested (blank if the motor isn't in motors.csv) |
| `motor_name` | str | Motor as named by the test |
| `esc` | str | ESC / driver used |
| `load` | str | Load: propeller, dynamometer brake, wheel ... |
| `device` | str | Test stand |
| `v_min_V` | float | Lowest supply voltage seen in the run |
| `v_max_V` | float | Highest supply voltage seen in the run |
| `rpm_max` | float | Highest speed reached, rpm |
| `rpm_per_V_at_max` | float | rpm / V at the top point (a loaded lower bound of KV) |
| `current_max_A` | float | Highest supply current |
| `power_elec_max_W` | float | Highest electrical input power |
| `power_mech_max_W` | float | Highest mechanical (shaft) power |
| `torque_max_Nm` | float | Highest shaft torque |
| `thrust_max_N` | float | Highest thrust (propeller tests) |
| `eff_max` | float | Best motor + ESC efficiency, 0-1 |
| `n_points` | int | Data points in the run |
| `url` | str | Link to the test |
| `prov_default` | enum (datasheet / reseller / forum / est / measured / third_party / press) | Provenance of the values |
| `source_ids` | list | Source ids from sources.csv |
| `date_checked` | date | YYYY-MM-DD the test was read |
| `notes` | str | Notes |

### computers.csv

| Column | Type | Meaning |
| --- | --- | --- |
| `id` | str | Unique slug, maker-part (e.g. pololu-37d-24v-19-enc). Never reuse or rename: projects refer to it |
| `manufacturer` | str | Maker |
| `part_number` | str | Maker's part number or SKU |
| `name` | str | Descriptive name |
| `vendor` | str | Where the price comes from (shop) |
| `kind` * | enum (dev_kit / module / carrier / sbc / mini_pc / accelerator) | dev_kit (module + carrier, ready to run), module (needs a carrier), carrier, sbc, mini_pc, accelerator (an add-on NPU) |
| `platform` * | enum (jetson_orin / jetson_thor / raspberry_pi_5 / rk3588 / qcs6490 / x86) | What software and cameras it takes (cameras.csv hosts) |
| `soc` * | str | SoC or CPU |
| `cpu` | str | Cores, type and clock (e.g. 6x Cortex-A78AE @ 1.7 GHz) |
| `cpu_big_cores` * | int | Fast cores (all cores when they are alike) |
| `cpu_little_cores` | int | Slow cores (Cortex-A55, Intel E-cores beside P-cores); 0 when all alike |
| `cpu_max_GHz` | float | Highest clock of the fast cores |
| `ram_GB` * | float | Memory as sold (blank for a barebone or a carrier) |
| `ram_type` | str | LPDDR5, DDR5 SO-DIMM (not included) ... |
| `gpu` | str | GPU |
| `ai_tops` | float | Maker's AI rating, TOPS (see ai_tops_precision) |
| `ai_tops_precision` | str | What the TOPS figure counts: sparse INT8, INT8, INT4 ... |
| `bench_gb6_single` * | float | Geekbench 6 single-core score (per-thread speed: scales CPU loads) |
| `bench_gb6_multi` | float | Geekbench 6 multi-core score |
| `bench_7zip_single_MIPS` | float | 7-Zip single-thread rating, MIPS (sbc-bench) |
| `bench_7zip_multi_MIPS` | float | 7-Zip all-threads rating, MIPS (sbc-bench) |
| `os_support` | str | Maker-supported operating systems |
| `ubuntu_2404` * | enum (yes / no / partly / VERIFY) | Runs Ubuntu 24.04 from the maker or Canonical (ROS 2 Jazzy's tier-1 OS) |
| `ros2_jazzy` | enum (yes / no / partly / VERIFY) | ROS 2 Jazzy binaries install natively |
| `csi_cameras` * | int | MIPI CSI-2 camera connectors (0 = USB cameras only) |
| `csi_lanes` | str | Lanes per connector |
| `hw_video_encoder` | enum (yes / no / partly / VERIFY) | Hardware video encoder |
| `usb3_ports` | int | USB 3 host ports (Type-A and host-capable Type-C) |
| `usb_notes` | str | Other USB |
| `display_out` | str | Display outputs |
| `network` | str | Ethernet, Wi-Fi |
| `expansion` | str | GPIO header, M.2, PCIe, CAN ... |
| `v_in_min_V` * | float | Lowest DC input voltage |
| `v_in_max_V` * | float | Highest DC input voltage |
| `power_idle_W` | float | Power at idle |
| `power_max_W` * | float | Power at full load (or the top power mode) |
| `power_modes` | str | Power modes / TDP settings |
| `size_mm` * | str | L x W x H as sold (with its cooler) |
| `mass_kg` | float | Mass |
| `price_usd` * | float | Unit price in USD at quantity 1 (converted prices are est) |
| `price_breaks` | str | Quantity breaks 'qty:usd;qty:usd' (e.g. 5:56.07;25:51.59) |
| `availability` | str | Stock seen on date_checked (in stock / out of stock / backorder / discontinued), or in hand (owned, Fernando's bench) |
| `prov_default` | enum (datasheet / reseller / forum / est / measured / third_party / press) | Provenance of every filled value unless overridden |
| `prov_overrides` | str | Exceptions: 'est: col, col; reseller: col' (column names of this table) |
| `source_ids` | list | Source ids from sources.csv, ';'-separated |
| `date_checked` | date | YYYY-MM-DD the values and price were last checked |
| `notes` | str | Anything else: caveats, VERIFY items, why rejected |
| `meas_stack_cpu_pct` | float | MEASURED CPU load of a named software stack, % of all cores (meas_notes says which) |
| `meas_power_W` | float | MEASURED power running that stack |
| `meas_date` | date | Date of the bench test |
| `meas_notes` | str | Stack, setup, log file |

### cameras.csv

| Column | Type | Meaning |
| --- | --- | --- |
| `id` | str | Unique slug, maker-part (e.g. pololu-37d-24v-19-enc). Never reuse or rename: projects refer to it |
| `manufacturer` | str | Maker |
| `part_number` | str | Maker's part number or SKU |
| `name` | str | Descriptive name |
| `vendor` | str | Where the price comes from (shop) |
| `kind` * | enum (module / stereo_kit / usb_camera / smart_camera / depth_camera / capture_board) | module (one sensor board), stereo_kit (two synced sensors sold together), usb_camera, smart_camera (does its own processing), depth_camera, capture_board (takes camera modules, feeds a host) |
| `sensor` * | str | Image sensor (blank for a capture board) |
| `shutter` * | enum (global / rolling / VERIFY) | global or rolling |
| `color` * | enum (color / mono / mono_stereo_color_center / VERIFY) | color, mono, or both (a mono stereo pair with a color camera) |
| `res_w_px` * | int | Full-resolution width, px |
| `res_h_px` * | int | Full-resolution height, px |
| `fps_full` * | float | Frame rate at full resolution (per sensor) |
| `fps_modes` | str | Other modes, 'WxH@fps; ...' (a stereo kit's side-by-side modes as sold) |
| `pixel_um` | float | Pixel pitch, um |
| `output_format` | str | RAW10, YUV, MJPEG ... |
| `interface` * | list | mipi_csi2, usb2, usb3, gmsl2, ethernet |
| `mipi_lanes` | int | MIPI lanes |
| `connector` | str | Connector |
| `hw_trigger` * | enum (yes / no / partly / VERIFY) | Takes a hardware trigger (frame sync) input that a host or MCU can drive |
| `trigger_notes` | str | Trigger pin, logic level, edge, minimum pulse, highest trigger rate |
| `strobe_out` | enum (yes / no / partly / VERIFY) | Gives an exposure / strobe output |
| `stereo_sync` | str | How two of them are synchronised |
| `baseline_mm` | float | Stereo baseline (fixed-baseline kits and depth cameras) |
| `fov_deg` | str | Field of view (H x V, or D) with the stock lens |
| `lens_mount` | str | M12, C/CS, fixed ... |
| `hosts` * | list | Platforms it works with (computers.csv platform) plus usb (any USB 3 host), gmsl_card, oak_ffc |
| `driver` | str | Driver: upstream, vendor per OS release ... |
| `latency_ms` | float | Published capture-to-host latency |
| `latency_notes` | str | How that latency was measured |
| `power_W` | float | Power |
| `size_mm` | str | Size |
| `mass_kg` | float | Mass |
| `price_usd` * | float | Unit price in USD at quantity 1 (converted prices are est) |
| `price_breaks` | str | Quantity breaks 'qty:usd;qty:usd' (e.g. 5:56.07;25:51.59) |
| `availability` | str | Stock seen on date_checked (in stock / out of stock / backorder / discontinued), or in hand (owned, Fernando's bench) |
| `prov_default` | enum (datasheet / reseller / forum / est / measured / third_party / press) | Provenance of every filled value unless overridden |
| `prov_overrides` | str | Exceptions: 'est: col, col; reseller: col' (column names of this table) |
| `source_ids` | list | Source ids from sources.csv, ';'-separated |
| `date_checked` | date | YYYY-MM-DD the values and price were last checked |
| `notes` | str | Anything else: caveats, VERIFY items, why rejected |
| `meas_latency_ms` | float | MEASURED trigger-to-application latency (glass to software) |
| `meas_fps` | float | MEASURED frame rate in the mode used |
| `meas_calib_drift_deg` | float | MEASURED stereo calibration drift between sessions, deg |
| `meas_date` | date | Date of the bench test |
| `meas_notes` | str | Setup, mode, log file |

### sources.csv

| Column | Type | Meaning |
| --- | --- | --- |
| `id` | str | Source id (S001 ...) |
| `kind` | enum (maker / reseller / forum / project / test_database / press) | maker (maker's page or datasheet), reseller, forum, project (own docs / purchase records), test_database (third-party bench data), press (news or review site) |
| `url` | str | Link (or a repo path for project sources) |
| `supports` | str | Which parts and values it supports |
| `date_checked` | date | YYYY-MM-DD |
| `notes` | str | Notes |
