/*
 * VEX V5 Override 2026-27: Driver Control
 *
 * LEFT STICK      drives the whole robot (push the way you want to go)
 * L1 / L2         arm up / down (hold)
 * R1 / R2         claw grab / release (tap)
 * X / A / B       arm to score / carry / pickup height
 * Y               slow mode on/off
 *
 * BEFORE EVERY MATCH: start with the ARM ALL THE WAY DOWN and the CLAW OPEN.
 * The program counts those as position zero for the arm and claw.
 */

#include "vex.h"

using namespace vex;

// =============================================================
// Devices: change ports and directions to match your robot
// =============================================================
brain Brain;
controller Controller1 = controller(primary);
competition Competition;

motor LeftFront  = motor(PORT2, ratio18_1, false);
motor LeftRear   = motor(PORT3, ratio18_1, false);
motor RightFront = motor(PORT4, ratio18_1, true);
motor RightRear  = motor(PORT5, ratio18_1, true);
motor_group LeftDrive  = motor_group(LeftFront, LeftRear);
motor_group RightDrive = motor_group(RightFront, RightRear);

motor Arm  = motor(PORT6, ratio36_1, false);  // positive spin should raise the arm
motor Claw = motor(PORT7, ratio36_1, false);  // positive spin should close the claw

motor *allMotors[] = {&LeftFront, &LeftRear, &RightFront, &RightRear, &Arm, &Claw};

// =============================================================
// Tuning: adjust these numbers instead of the code below
// =============================================================
// Drive
const double MAX_VOLTS  = 12.0;  // full motor power
const double DEADBAND   = 0.05;  // ignore the first 5% of stick travel (stops drift)
const double CURVE      = 0.6;   // 0 = straight line, 1 = full cubic. Higher = finer slow control
const double TURN_SCALE = 0.75;  // turning is a bit gentler than driving
const double ACCEL_STEP = 3.0;   // max volts added per loop. 12 turns the limit off
const double SLOW_SPEED = 0.4;   // speed multiplier in slow mode

// Arm, measured in motor degrees from the bottom
const double ARM_MIN    = 0;
const double ARM_MAX    = 600;
const double ARM_PICKUP = 0;
const double ARM_CARRY  = 150;
const double ARM_SCORE  = 450;
const double ARM_VOLTS  = 10.0;  // arm speed when moved by hand
const bool   AUTO_LIFT  = true;  // lift to carry height right after a grab

// Claw
const double CLAW_SQUEEZE_VOLTS = 10.0;  // power while closing
const double CLAW_HOLD_VOLTS    = 3.0;   // lighter grip once closed, keeps the motor cool
const double CLAW_OPEN_POS      = 0;     // open position in motor degrees

// Warnings
const double HOT_TEMP_C = 55;  // V5 motors start cutting power when they get this hot

const int LOOP_MS = 20;

// =============================================================
// Helpers
// =============================================================

// Fires once when a button is first pressed, not every loop while it's held
struct ButtonTap {
  bool wasDown = false;
  bool tapped(bool isDown) {
    bool fired = isDown && !wasDown;
    wasDown = isDown;
    return fired;
  }
};

// Turns a raw stick reading (-100 to 100) into a smooth value (-1 to 1)
double shapeStick(int raw) {
  double x = raw / 100.0;
  if (fabs(x) < DEADBAND) return 0;

  // Rescale so the output starts at 0 right after the deadband, with no jump
  double sign = (x > 0) ? 1.0 : -1.0;
  x = sign * (fabs(x) - DEADBAND) / (1.0 - DEADBAND);

  // Blend a straight line with a cubic: gentle near the center, full power at the edge
  return CURVE * x * x * x + (1.0 - CURVE) * x;
}

// Lets a side slow down instantly but limits how fast it speeds up
double limitAccel(double target, double previous) {
  if (target * previous < 0) previous = 0;            // changing direction: start from zero
  if (fabs(target) <= fabs(previous)) return target;  // slowing down: no limit
  if (target > previous) return fmin(target, previous + ACCEL_STEP);
  return fmax(target, previous - ACCEL_STEP);
}

void driveSide(motor_group &side, double volts) {
  if (fabs(volts) < 0.05) {
    side.stop();  // uses brake mode, so the robot stops instead of rolling
  } else {
    side.spin(directionType::fwd, volts, voltageUnits::volt);
  }
}

void moveArmTo(double degrees) {
  // "false" means don't wait: the robot keeps driving while the arm moves
  Arm.spinToPosition(degrees, rotationUnits::deg, 100, velocityUnits::pct, false);
}

double hottestMotor() {
  double hottest = 0;
  for (motor *m : allMotors) {
    hottest = fmax(hottest, m->temperature(temperatureUnits::celsius));
  }
  return hottest;
}

// Writes one controller screen row, but only if the text changed.
// The controller screen updates slowly over the radio, so fewer writes = no flicker.
char shownRows[3][24] = {"", "", ""};
void showRow(int row, const char *text) {
  char padded[24];
  snprintf(padded, sizeof(padded), "%-16s", text);
  if (strcmp(padded, shownRows[row]) == 0) return;
  Controller1.Screen.setCursor(row + 1, 1);
  Controller1.Screen.print("%s", padded);
  strcpy(shownRows[row], padded);
}

// =============================================================
// Competition phases
// =============================================================

void pre_auton() {
  LeftDrive.setStopping(brakeType::brake);
  RightDrive.setStopping(brakeType::brake);
  Arm.setStopping(brakeType::hold);   // arm stays where you leave it
  Claw.setStopping(brakeType::hold);

  Arm.setPosition(0, rotationUnits::deg);   // arm must start fully down
  Claw.setPosition(0, rotationUnits::deg);  // claw must start fully open

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Override 2026/27 Driver Control");
  Brain.Screen.newLine();
  Brain.Screen.print("Start: arm DOWN, claw OPEN");
  Brain.Screen.newLine();
  Brain.Screen.print("L stick drive | L1/L2 arm | R1/R2 claw");
  Brain.Screen.newLine();
  Brain.Screen.print("X score | A carry | B pickup | Y slow");
}

void autonomous() {
  // Add your autonomous routine here. It runs during the autonomous period.
}

void usercontrol() {
  ButtonTap grabBtn, releaseBtn, scoreBtn, carryBtn, pickupBtn, slowBtn;

  bool slowMode = false;
  bool armWasManual = false;
  bool hotWarned = false;
  double hottest = 0;
  double leftVolts = 0;
  double rightVolts = 0;

  enum ClawState { CLAW_OPEN, CLAW_CLOSING, CLAW_GRIPPING };
  ClawState clawState = CLAW_OPEN;
  int clawTicks = 0;

  int tick = 0;
  int screenRow = 0;

  Controller1.Screen.clearScreen();
  for (int i = 0; i < 3; i++) shownRows[i][0] = '\0';

  while (true) {
    // ---------------- Drive: one-stick arcade ----------------
    double forward = shapeStick(Controller1.Axis3.position(percent));
    double turn    = shapeStick(Controller1.Axis4.position(percent)) * TURN_SCALE;

    double left  = forward + turn;
    double right = forward - turn;

    // If a side would go past 100%, shrink both sides equally.
    // This keeps the direction you pushed instead of cutting off one side.
    double biggest = fmax(fabs(left), fabs(right));
    if (biggest > 1.0) {
      left /= biggest;
      right /= biggest;
    }

    if (slowBtn.tapped(Controller1.ButtonY.pressing())) {
      slowMode = !slowMode;
      Controller1.rumble("-");
    }
    double speed = slowMode ? SLOW_SPEED : 1.0;

    leftVolts  = limitAccel(left * speed * MAX_VOLTS, leftVolts);
    rightVolts = limitAccel(right * speed * MAX_VOLTS, rightVolts);
    driveSide(LeftDrive, leftVolts);
    driveSide(RightDrive, rightVolts);

    // ---------------- Arm ----------------
    bool armUp   = Controller1.ButtonL1.pressing();
    bool armDown = Controller1.ButtonL2.pressing();
    bool scoreTap  = scoreBtn.tapped(Controller1.ButtonX.pressing());
    bool carryTap  = carryBtn.tapped(Controller1.ButtonA.pressing());
    bool pickupTap = pickupBtn.tapped(Controller1.ButtonB.pressing());

    if (armUp || armDown) {
      // Manual control always wins over presets
      double pos = Arm.position(rotationUnits::deg);
      if (armUp && pos < ARM_MAX) {
        Arm.spin(directionType::fwd, ARM_VOLTS, voltageUnits::volt);
      } else if (armDown && pos > ARM_MIN) {
        Arm.spin(directionType::rev, ARM_VOLTS, voltageUnits::volt);
      } else {
        Arm.stop();  // reached a limit: hold there
      }
      armWasManual = true;
    } else {
      if (armWasManual) {
        Arm.stop();  // button released: hold the arm where it is
        armWasManual = false;
      }
      if (scoreTap)       moveArmTo(ARM_SCORE);
      else if (carryTap)  moveArmTo(ARM_CARRY);
      else if (pickupTap) moveArmTo(ARM_PICKUP);
    }

    // ---------------- Claw ----------------
    if (grabBtn.tapped(Controller1.ButtonR1.pressing())) {
      clawState = CLAW_CLOSING;
      clawTicks = 0;
    }
    if (releaseBtn.tapped(Controller1.ButtonR2.pressing())) {
      clawState = CLAW_OPEN;
      Claw.spinToPosition(CLAW_OPEN_POS, rotationUnits::deg, 100, velocityUnits::pct, false);
    }

    if (clawState == CLAW_CLOSING) {
      Claw.spin(directionType::fwd, CLAW_SQUEEZE_VOLTS, voltageUnits::volt);
      clawTicks++;

      // Once the claw stops moving, it has squeezed something (or closed fully)
      bool stalled  = clawTicks > 8 && fabs(Claw.velocity(velocityUnits::pct)) < 5;
      bool timedOut = clawTicks > 75;  // safety: 1.5 seconds max
      if (stalled || timedOut) {
        clawState = CLAW_GRIPPING;
        Claw.spin(directionType::fwd, CLAW_HOLD_VOLTS, voltageUnits::volt);
        Controller1.rumble(".");

        if (AUTO_LIFT && !armWasManual && Arm.position(rotationUnits::deg) < ARM_CARRY - 20) {
          moveArmTo(ARM_CARRY);
        }
      }
    }

    // ---------------- Warnings (once a second) ----------------
    if (tick % 50 == 0) {
      hottest = hottestMotor();
      if (hottest >= HOT_TEMP_C && !hotWarned) {
        Controller1.rumble("---");
        hotWarned = true;
      }
      if (hottest < HOT_TEMP_C - 5) hotWarned = false;
    }

    // ---------------- Controller screen (one row every 100 ms) ----------------
    if (tick % 5 == 0) {
      char text[24];
      if (screenRow == 0) {
        snprintf(text, sizeof(text), "%s", slowMode ? "SLOW MODE" : "FULL SPEED");
      } else if (screenRow == 1) {
        const char *claw = (clawState == CLAW_OPEN)    ? "open"
                         : (clawState == CLAW_CLOSING) ? "closing"
                                                       : "GRIP";
        snprintf(text, sizeof(text), "Claw %s", claw);
      } else {
        snprintf(text, sizeof(text), "Bat%3d%% Mtr%3dC%s", (int)Brain.Battery.capacity(),
                 (int)hottest, hottest >= HOT_TEMP_C ? "!" : "");
      }
      showRow(screenRow, text);
      screenRow = (screenRow + 1) % 3;
    }

    tick++;
    wait(LOOP_MS, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
