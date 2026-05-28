# REPORT OF ALL INSTANCES OF REMOVED ELEMENTS FROM THE MIGRATION TO SYSTEMCORE IN 2027
# GOT THE GITHUB AI TO SCAN THE ENTIRE REPOSITORY FOR CHANGES LISTED IN WPILIB'S SYSTEMCORE CHANGELOG FOR 2027
# https://docs.wpilib.org/en/2027/docs/yearly-overview/removed-features.html <- changelog is here

# SystemCore Migration Audit Report
**Date:** April 8, 2026  
**Project:** FRC-2026-Final  
**Target:** SystemCore (RoboRIO Replacement for 2027)

# LINK TO Team 6328's experience using the SystemCore: 
# https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-systemcore-alpha-build-thread/503556/22?page=2
---

## Executive Summary

This report documents all usage of **removed features** that will not be available on SystemCore. Your codebase is **mostly future-compatible**, with only **SmartDashboard** and **Telemetry** components that need migration to alternative solutions.

### Quick Metrics
- ✅ **No Relay, Servo, Counter, Ultrasonic, Analog I/O usage** (hardware not supported)
- ✅ **No RamseteController usage** (use LTV Unicycle Controller instead)
- ✅ **No PIDCommand, ProfiledPIDCommand, or TrapezoidProfileCommand**
- ⚠️ **SmartDashboard used in 3 files** (needs migration to Elastic/AdvantageScope)
- ⚠️ **robotInit() utilized** (works fine but can be optimized)
- ✅ **No MathUtil.clamp() usage** (or already commented out)

---

## Detailed Findings

### 1. ⚠️ SmartDashboard Usage (REQUIRES MIGRATION)

**Status:** 3 files use SmartDashboard for telemetry  
**Impact:** SystemCore does not use NT3; SmartDashboard requires NT3 protocol  
**Replacement:** Use **Elastic** (driver dashboard) + **AdvantageScope** (debugging/visualization)

## WE WILL REPLACE THE MAJORITY OF SMARTDASHBOARD WITH ADVANTAGESCOPE LOGGING


#### Affected Files:

##### **Robot.java**
- **Import:** `edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;` (line 19)
- **Usages:**
  - Line 85: `SmartDashboard.putNumber("shooter speed", ...)`
  - Lines 86-87: Commented out pivot speed and intake speed
  - Line 93: `SmartDashboard.putNumber("shooterMotor1 subsystem rpmControl", ...)`
  - Line 97: `SmartDashboard.putNumber("intake motor velocity", ...)`
  - Line 99: `SmartDashboard.putNumber("pivot amount", ...)`
  - Line 100: `SmartDashboard.putNumber("drivetrain yaw", ...)`
  - Line 101: `SmartDashboard.putNumber("drivetrain x", ...)`
  - Line 102: `SmartDashboard.putNumber("drivetrain y", ...)`
  - Line 103: `SmartDashboard.putNumber("drivetrain distance to hub", ...)`

**Recommendation:** 
- Remove SmartDashboard calls
- Use `Logger.recordOutput()` (from AdvantageKit) instead for AdvantageScope logging
- Elastic will handle NT4 driver station display

---

##### **Telemetry.java**
- **Import:** `edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;` (line 17)
- **Usages:**
  - Lines 53-75: Mechanism2d visualization setup (still valid)
  - Line 121: `SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);`

**Note:** `Mechanism2d` and `MechanismLigament2d` are still supported on SystemCore  
**Recommendation:** 
- Keep Mechanism2d code (it's valid)
- Replace the SmartDashboard.putData() call with mechanism publishing via Logger or NT4

---

##### **Pivoting.java** (Command)
- **Import:** `edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;` (line 4)
- **Usage:**
  - Line 35: `SmartDashboard.putNumber("pivot location", ...)`

**Recommendation:** 
- Replace with `Logger.recordOutput("Pivot/location", ...)` from AdvantageKit

---

### 2. ✅ robotInit() Method

**File:** `Robot.java` (line 54)  
**Status:** Currently implemented correctly  
**Content:**
```java
@Override
public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    Pathmaster.warmupCommand();
}
```

**Note:** robotInit() is not deprecated; it still works on SystemCore.  
**Future Optimization:** Could move initialization logic into the Robot constructor, but not required.

---

### 3. ✅ MathUtil.clamp() Usage

**File:** `Shooter.java` (line 238)  
**Status:** Currently commented out (no active usage)
```java
//         correction = MathUtil.clamp(correction, -0.2, 0.2);
```

**Action:** No migration needed (not in use)  
**Note if reactivating:** Replace `MathUtil.clamp()` with `Math.clamp()` when uncommented

---

### 4. ✅ NetworkTables Usage

**Status:** ✅ Your code uses **NetworkTables 4 (NT4)**  
**Files:** 
- `Telemetry.java` - Uses NT4 publishers/subscribers (correct)
- `LimelightHelpers.java` - Uses NT4 for Limelight communication (correct)
- Other files using Logger from AdvantageKit (correct)

**No migration needed** - NT4 is fully supported on SystemCore

---

### 5. ✅ Hardware Classes (Relay, Servo, Counter, etc.)

**Status:** ✓ No usage found  
**Verified:**
- ✅ No Relay imports or usage
- ✅ No Servo imports or usage
- ✅ No Counter imports or usage
- ✅ No Ultrasonic imports or usage
- ✅ No Analog Output/Gyro usage
- ✅ No SPI usage

---

### 6. ✅ Control Commands (Ramsete, PIDCommand, etc.)

**Status:** ✓ No deprecated control commands found  
**Verified:**
- ✅ No RamseteController usage
- ✅ No PIDCommand usage
- ✅ No ProfiledPIDCommand usage
- ✅ No TrapezoidProfileCommand usage
- ✅ Uses modern PIDController/ProfiledPIDController pattern

---

### 7. ✅ build.gradle Configuration

**Status:** ✓ Uses GradleRIO 2026.2.1 (SystemCore compatible)

**Current Configuration:**
```groovy
plugins {
    id "edu.wpi.first.GradleRIO" version "2026.2.1"
}
```

**Deployment Target:** RoboRIO (will transition to SystemCore in 2027)

**Vendor Dependencies Found:**
- AdvantageKit ✅ (fully supported)
- PathplannerLib ✅ (supported)
- Phoenix 6 ✅ (supported)
- REVLib ✅ (supported)
- PhotonLib ✅ (supported)

**No deprecated dependencies detected**

---

## Migration Checklist for 2027 SystemCore Transition

### High Priority (Must Fix)
- [ ] **Replace SmartDashboard.putNumber() calls** in Robot.java with `Logger.recordOutput()`
  - Affected lines: 85, 93, 97, 99-103
  - Files: `Robot.java`, `Telemetry.java`, `Pivoting.java`

### Medium Priority (Best Practice)
- [ ] **Test Mechanism2d visualization** with AdvantageScope NT4 publisher
  - Ensure `SmartDashboard.putData()` for mechanisms is properly logged

### Low Priority (Optional)
- [ ] Consider moving initialization from `robotInit()` to Robot constructor
- [ ] Document telemetry export process with Elastic/AdvantageScope

---

## Recommended Migration Path

### Step 1: Add AdvantageScope/Elastic configuration
```java
// Already using AdvantageKit correctly!
Logger.recordOutput("Shooter/RPM", rpm);
Logger.recordOutput("Drivetrain/Pose", pose);
```

### Step 2: Replace SmartDashboard calls
**Before:**
```java
SmartDashboard.putNumber("shooter speed", RobotContainer.shooterSpeed);
```

**After:**
```java
Logger.recordOutput("Shooter/TargetSpeed", RobotContainer.shooterSpeed);
```

### Step 3: Update GradleRIO version (when 2027 season tools available)
```groovy
id "edu.wpi.first.GradleRIO" version "2027.x.x"  // Will be specified by WPILib
```

### Step 4: Verify all vendor libraries are SystemCore compatible
- Phoenix 6 ✅
- REVLib ✅
- AdvantageKit ✅
- PathPlanner ✅

---

## Files Summary

| File | Status | Action Required |
|------|--------|-----------------|
| `build.gradle` | ✅ | None |
| `Robot.java` | ⚠️ | Remove SmartDashboard imports/calls |
| `Telemetry.java` | ⚠️ | Keep Mechanism2d, remove SmartDashboard.putData() |
| `Pivoting.java` | ⚠️ | Replace SmartDashboard with Logger |
| `Shooter.java` | ✅ | None (MathUtil.clamp commented out) |
| `LimelightHelpers.java` | ✅ | None (NT4 usage correct) |
| `VisionSubsystem.java` | ✅ | None |
| All other files | ✅ | None |

---

## References

- [WPILib 2027 SystemCore Announcement](https://docs.wpilib.org/en/latest/)
- [AdvantageKit Documentation](https://github.com/Mechanical-Advantage/AdvantageKit)
- [Elastic Documentation](https://app.elastic.co/)
- [AdvantageScope Repository](https://github.com/Mechanical-Advantage/AdvantageScope)

---

**Report Generated:** April 8, 2026  
**Next Review:** After 2027 GradleRIO release
