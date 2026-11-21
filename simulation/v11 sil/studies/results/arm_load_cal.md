# Arm load calibration on the detailed arm (studies/arm_load_cal.py)

Each seed: a detailed arm (its own truth), the procedure with the simulated person, then the same arm tested with the driver three ways: legacy (before Nov 20: no hold trim, no reflex) / nominal (datasheet load model, trim, reflex allowed) / calibrated. est throughout: no servo measured yet.

## Seed 0

| | legacy | nominal | calibrated |
| --- | --- | --- | --- |
| still torque error, N m rms | 0.280 | 0.279 | 0.183 |
| 0.20 kg weight's force error, N | 2.77 | 2.75 | 1.88 |
| false reflexes in 25 tidy-speed moves | 0 | 24 | 0 |
| box press: contact force at the reflex, N | - | 20.5 | 11.9 |
| box press: delay after first contact, s | - | 0.63 | 0.42 |
| box press: peak force before the reflex (or all), N | 25.2 | 20.7 | 11.9 |
| sideways push: force at the reflex, N | - | 8.9 | 5.5 |
| reach: tool error, mm rms | 10.2 | 5.8 | 4.7 |
| reach: vertical, mm rms | 3.1 | 1.4 | 2.3 |

Calibration: still torque 0.085 N m rms (nominal 0.290), weight 1.18 N (nominal 2.89), heights 1.0 mm (nominal 3.1); 5981 records, refit of records_seed0.pkl

## Seed 1

| | legacy | nominal | calibrated |
| --- | --- | --- | --- |
| still torque error, N m rms | 0.228 | 0.229 | 0.141 |
| 0.20 kg weight's force error, N | 2.60 | 2.54 | 1.83 |
| false reflexes in 25 tidy-speed moves | 0 | 23 | 0 |
| box press: contact force at the reflex, N | - | 19.2 | 10.6 |
| box press: delay after first contact, s | - | 0.62 | 0.38 |
| box press: peak force before the reflex (or all), N | 24.2 | 19.8 | 10.7 |
| sideways push: force at the reflex, N | - | 8.4 | 5.9 |
| reach: tool error, mm rms | 6.0 | 7.3 | 5.9 |
| reach: vertical, mm rms | 1.2 | 0.4 | 0.7 |

Calibration: still torque 0.091 N m rms (nominal 0.293), weight 1.09 N (nominal 2.40), heights 0.6 mm (nominal 1.3); 6004 records, refit of records_seed1.pkl

## Seed 2

| | legacy | nominal | calibrated |
| --- | --- | --- | --- |
| still torque error, N m rms | 0.185 | 0.183 | 0.103 |
| 0.20 kg weight's force error, N | 2.46 | 2.36 | 1.72 |
| false reflexes in 25 tidy-speed moves | 0 | 23 | 0 |
| box press: contact force at the reflex, N | - | 19.7 | 12.4 |
| box press: delay after first contact, s | - | 0.65 | 0.44 |
| box press: peak force before the reflex (or all), N | 24.1 | 20.8 | 12.4 |
| sideways push: force at the reflex, N | - | 7.4 | 4.8 |
| reach: tool error, mm rms | 6.5 | 3.5 | 4.2 |
| reach: vertical, mm rms | 2.4 | 1.3 | 0.6 |

Calibration: still torque 0.082 N m rms (nominal 0.271), weight 1.07 N (nominal 2.53), heights 0.6 mm (nominal 3.2); 5915 records, refit of records_seed2.pkl

