#IM DELETING THIS SHIT

# Hybrid ProfiledPIDController + SparkClosedLoopController Design

## Overview
Successfully fused the **ProfiledPIDController** (as primary controller) with the **SparkClosedLoopController** (as feedback correction) for the Pivot subsystem. This hybrid approach provides smooth trapezoidal motion profiles while maintaining passive feedback correction.

---

## Architecture

### Primary Controller: ProfiledPIDController
- **Role**: Main control loop in the `Pivoting` command
- **Features**: 
  - Generates smooth trapezoidal motion profiles
  - Respects velocity and acceleration constraints from `IntakeConstants`
  - Calculates desired output based on position error
  - Runs at 50 Hz (0.02s period)

### Secondary Controller: SparkClosedLoopController
- **Role**: Passive feedback correction in the `Pivot` subsystem
- **Features**:
  - Small proportional gain (`FEEDBACK_KP = 0.04`)
  - Always active on the motor controller
  - Provides low-level corrections for drift/disturbances
  - Prevents oscillation with zero I and D gains

---

## Control Flow

```
Pivoting Command
│
├─ initialize(): Reset ProfiledPIDController to current position
│
├─ execute() (runs every cycle):
│   ├─ Read current pivot position
│   ├─ Calculate desired output using ProfiledPIDController
│   │   └─ Respects acceleration/velocity constraints
│   │
│   └─ Call pivot.setHybridOutput(profiledOutput)
│       │
│       └─ Pivot Subsystem
│           ├─ Clamp output to [-1.0, 1.0]
│           ├─ Send to motor: pivotMotor.set(clampedOutput)
│           │
│           └─ SparkClosedLoopController (hardware level)
│               └─ Provides passive P-correction (FEEDBACK_KP = 0.04)
│
└─ end(): Stop motor (holds via hardware)
```

---

## Key Methods

### In `Pivot` Subsystem:

```java
/**
 * Apply hybrid control output
 * The ProfiledPIDController (primary) drives motion,
 * while SparkClosedLoopController (secondary) provides passive correction
 */
public void setHybridOutput(double profiledOutput) {
    double clampedOutput = Math.max(-1.0, Math.min(1.0, profiledOutput));
    pivotMotor.set(clampedOutput);
}

public void setProfiledPIDController(ProfiledPIDController controller) {
    this.profiledPIDController = controller;
}
```

### In `Pivoting` Command:

```java
@Override
public void execute() {
    double currentPosition = pivot.getPivotPosition();
    double profiledOutput = pivotController.calculate(currentPosition, goalState);
    pivot.setHybridOutput(profiledOutput);  // Send to hybrid controller
}

@Override
public void initialize() {
    pivotController.reset(pivot.getPivotPosition());  // Start fresh
}
```

---

## Advantages of Hybrid Approach

| Aspect | Benefit |
|--------|---------|
| **Motion Profile** | Smooth, constraint-respecting motion with controlled accel/decel |
| **Drift Correction** | SparkClosedLoopController automatically compensates for small disturbances |
| **Robustness** | Two layers of control provide redundancy and stability |
| **Tuning Flexibility** | ProfiledPIDController gains can be tuned independently from feedback gains |
| **Hardware Integration** | Leverages REV's internal controller for low-latency corrections |

---

## Configuration Parameters

### ProfiledPIDController (from `IntakeConstants`):
- **kP**: `pivotkP` (primary control gain)
- **kI**: `pivotkI` (integral correction)
- **kD**: `pivotkD` (derivative damping)
- **Max Velocity**: `kMaxPivotVelocity` (1.0 rot/sec)
- **Max Acceleration**: `kMaxPivotAcceleration` (1.0 rot/sec²)

### SparkClosedLoopController (Feedback Layer):
- **FEEDBACK_KP**: `0.04` (small drift correction)
- **FEEDBACK_KI**: `0.0` (no integral to avoid windup)
- **FEEDBACK_KD**: `0.0` (no derivative to reduce noise)
- **Output Range**: `[-1.0, 1.0]` (full motor range)

---

## Unused Methods Removed

The following methods were removed as they are no longer needed:
- ✅ `spinPivotPIDAmped()` - Redundant with hybrid control
- ✅ `spinPivotPID()` - Redundant with hybrid control
- ✅ `spinPivotDutyAmped()` - Logic bug (double set call)
- ✅ `setOutputLimits()` - Unused clamping logic
- ✅ `setTopLimit()` - Unused limit setter
- ✅ `setBottomLimit()` - Unused limit setter

---

## Testing Recommendations

1. **Smooth Motion**: Verify pivot accelerates/decelerates smoothly at start/end
2. **Constraint Compliance**: Ensure max velocity and acceleration are respected
3. **Disturbance Rejection**: Test feedback correction by applying load during motion
4. **Position Accuracy**: Verify final position matches target within tolerance (0.05 rotations)
5. **Tuning**: Adjust `pivotkP`, `pivotkI`, `pivotkD` or `FEEDBACK_KP` if needed
