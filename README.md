# VEX-Optimized
We da best music

Driver control code for a VEX V5 robot with a 4-motor drivetrain, an arm, and a claw. Written in C++ for VEXcode V5.

Based on [imredavid64-glitch/VEX-V5-Override-2026-27](https://github.com/imredavid64-glitch/VEX-V5-Override-2026-27), rewritten with one-stick driving, claw and arm controls, and the competition template.

## Controls

| Finger | Button | What it does |
|---|---|---|
| Left thumb | Left stick | Drive. Push the way you want to go: up = forward, sideways = turn, diagonal = curve |
| Left index | L1 (hold) | Arm up |
| Left index | L2 (hold) | Arm down |
| Right index | R1 (tap) | Claw grab. Squeezes until it holds something, then auto-lifts to carry height |
| Right index | R2 (tap) | Claw release |
| Right thumb | X (tap) | Arm to score height |
| Right thumb | A (tap) | Arm to carry height |
| Right thumb | B (tap) | Arm to pickup height (bottom) |
| Right thumb | Y (tap) | Slow mode on/off |

Rumble feedback: short buzz = claw gripped, long buzz = slow mode toggled, three long buzzes = a motor is overheating.

Controller screen: row 1 speed mode, row 2 claw state, row 3 battery and hottest motor temperature.

## Before every match

Start with the **arm all the way down** and the **claw fully open**. The program treats those as zero, and the arm limits and presets are measured from there.

## Setup

1. In VEXcode V5, create a new C++ project (competition template is fine).
2. Replace its `src/main.cpp` with this repo's `src/main.cpp`.
3. Check the motor ports at the top of the file match your robot:
   - Left drive: ports 2, 3. Right drive: ports 4, 5 (reversed)
   - Arm: port 6. Claw: port 7
4. If the arm goes down when you press L1, or the claw opens when you press R1, flip that motor's `false` to `true`.
5. Tune the arm heights (`ARM_CARRY`, `ARM_SCORE`, `ARM_MAX`): drive the arm to each height, read the arm motor's position on the brain's device screen, and type the numbers in.

## Tuning

All tuning values are at the top of `main.cpp`.

| Setting | Default | Effect |
|---|---|---|
| `DEADBAND` | 0.05 | Stick movement ignored near center. Raise if the robot creeps |
| `CURVE` | 0.6 | 0 = linear, 1 = very soft near center. Lower if driving feels sluggish |
| `TURN_SCALE` | 0.75 | Max turning speed. Raise for faster spins |
| `ACCEL_STEP` | 3.0 | How fast the robot speeds up. 12 = no limit |
| `SLOW_SPEED` | 0.4 | Speed in slow mode |
| `CLAW_HOLD_VOLTS` | 3.0 | Grip strength after closing. Raise if objects slip out |
| `AUTO_LIFT` | true | Lift to carry height after a grab |
