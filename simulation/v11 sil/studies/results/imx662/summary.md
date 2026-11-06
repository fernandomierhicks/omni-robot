## Tags (pupil-apriltags, margin >= 30; recall against the still global-shutter frame from the same pose)

Exposure 4 ms. Each cell: recall / position error p50 (p90) in % of range / orientation flips (the pose ambiguity's other solution, normal > 30 deg off) / wrong ids.

| motion | readout 0 ms | readout 16 ms | readout 33 ms |
| --- | --- | --- | --- |
| still | 1.00 / 1.01 (1.26) / 3 / 0 | 1.00 / 0.99 (1.27) / 3 / 0 | 1.00 / 0.97 (1.29) / 2 / 0 |
| fwd0.5 | 1.00 / 0.91 (1.26) / 4 / 0 | 1.00 / 0.80 (1.15) / 3 / 0 | 1.00 / 0.57 (1.00) / 3 / 0 |
| fwd1 | 1.00 / 0.86 (1.30) / 4 / 0 | 1.00 / 0.58 (1.05) / 4 / 0 | 1.00 / 0.81 (1.15) / 4 / 0 |
| fwd2 | 1.00 / 0.75 (1.16) / 5 / 0 | 1.00 / 0.91 (1.47) / 5 / 0 | 1.00 / 2.39 (3.18) / 4 / 0 |
| side1 | 1.00 / 0.55 (1.20) / 3 / 0 | 1.00 / 0.90 (1.35) / 3 / 0 | 1.00 / 1.51 (1.75) / 2 / 0 |
| spin60 | 1.00 / 0.85 (1.99) / 4 / 0 | 1.00 / 1.56 (2.34) / 5 / 0 | 0.97 / 2.58 (2.77) / 4 / 0 |
| spin120 | 0.62 / 4.45 (6.38) / 0 / 0 | 0.65 / 5.02 (5.89) / 1 / 0 | 0.65 / 6.25 (7.11) / 1 / 0 |
| spin180 | 0.32 / 8.56 (8.65) / 0 / 0 | 0.32 / 9.08 (9.14) / 0 / 0 | 0.35 / 11.00 (11.13) / 0 / 0 |
| fwd1_wobble | 1.00 / 0.82 (1.15) / 5 / 0 | 1.00 / 0.91 (1.56) / 3 / 0 | 1.00 / 1.39 (2.47) / 0 / 0 |

Exposure 12 ms. Each cell: recall / position error p50 (p90) in % of range / orientation flips (the pose ambiguity's other solution, normal > 30 deg off) / wrong ids.

| motion | readout 0 ms | readout 16 ms | readout 33 ms |
| --- | --- | --- | --- |
| still | 1.00 / 1.02 (1.30) / 2 / 0 | 1.00 / 0.99 (1.28) / 3 / 0 | 1.00 / 0.98 (1.30) / 2 / 0 |
| fwd1 | 1.00 / 0.65 (1.08) / 4 / 0 | 1.00 / 0.60 (1.38) / 4 / 0 | 1.00 / 1.10 (2.26) / 5 / 0 |
| fwd2 | 0.86 / 0.51 (0.81) / 2 / 0 | 0.89 / 1.41 (1.62) / 3 / 0 | 0.86 / 1.77 (3.27) / 2 / 0 |
| side1 | 1.00 / 2.17 (4.13) / 5 / 0 | 1.00 / 2.77 (4.17) / 4 / 0 | 1.00 / 2.79 (4.42) / 3 / 0 |
| spin120 | 0.00 / - / 0 / 0 | 0.00 / - / 0 / 0 | 0.00 / - / 0 / 0 |
| spin180 | 0.00 / - / 0 / 0 | 0.00 / - / 0 / 0 | 0.00 / - / 0 / 0 |
| fwd1_wobble | 0.73 / 0.89 (3.41) / 4 / 0 | 0.73 / 1.19 (4.97) / 4 / 0 | 0.78 / 1.83 (5.48) / 4 / 0 |

## Optical flow (LK + essential matrix, 30 fps, 4 starts x 11 pairs)

Each cell: yaw-rate error deg/s p50 over the pairs of all starts (rotation flips past 90 deg/frame counted as failures) / direction-of-travel error deg p50 / failed pairs.

| motion | readout 0 ms | readout 16 ms | readout 33 ms |
| --- | --- | --- | --- |
| fwd0.5 | 1.6 / 121 / 8 | 2.0 / 79 / 6 | 1.5 / 9 / 3 |
| fwd1 | 2.0 / 8 / 2 | 1.6 / 11 / 4 | 1.5 / 7 / 3 |
| fwd2 | 2.2 / 3 / 0 | 1.6 / 3 / 0 | 1.5 / 3 / 0 |
| side1 | 1.7 / 3 / 6 | 1.2 / 3 / 10 | 1.9 / 3 / 7 |
| spin60 | 2.7 / 166 / 1 | 3.1 / 171 / 2 | 3.7 / 151 / 0 |
| spin180 | 101.4 / 61 / 11 | 54.4 / 58 / 9 | 34.6 / 91 / 5 |
| fwd1_wobble | 2.9 / 8 / 2 | 8.6 / 36 / 3 | 16.6 / 52 / 2 |
