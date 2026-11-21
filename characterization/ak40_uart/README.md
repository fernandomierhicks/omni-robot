# AK40-10 UART bench (Nov 19)

PC-side probes of one AK40-10 over its 3-pin serial port through CubeMars' R-link (FTDI, 921600 baud), to judge
whether UART was worth using on the robot. **Decision: the robot stays on CAN** (Fernando, Nov 19). Results and
the reasoning are in [components/ak40-10/README.md](../../components/ak40-10/README.md) "UART".

Run with the sim venv (`C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe`, pyserial). Close the CubeMars
tool first (it holds the port).

| Script | What | Motion |
| --- | --- | --- |
| `ak_uart_probe.py COM7 [n]` | Decodes one `COMM_GET_VALUES` (0x45) and times polled round trips (full and masked `GET_VALUES_SETUP`, 0x13). The ~16 ms round trip is the FTDI latency timer, not the motor | None |
| `ak_uart_rate.py COM7` | Pipelined requests at 200–2000 Hz, counts valid replies (the motor's sustainable telemetry rate) | None |
| `ak_uart_step.py COM7 out.csv [gap_us]` | Every 1 ms a `COMM_SET_RPM` + a masked query tagged by a rotating mask (echoed) so dropped replies are located; 0 → 10 rad/s output at 0.2 s, 0 at 0.8 s, coast at 1.3 s | **Spins the motor** (free, gently): attended |

`step_2026-11-19.csv`: the step run (1 ms rows aligned to the command: commanded and measured speed, Iq, input
current, duty, temperatures, bus voltage, fault).

Notes: the full `GET_VALUES` reply is 90 bytes (payload 0x55), longer than the manual's; only the documented head
fields are trusted. Mask bit n of the manual's table is `1 << (n-1)`. CRC is CRC-16/XMODEM over the payload.
