# Drive Subsystem

## Overview

This directory contains the swerve drive implementation and odometry/target prediction systems.

## Files

- **SwerveSubsystem.java** - CTRE Phoenix 6 swerve drivetrain wrapper
- **SwerveConstants.java** - Drive configuration parameters
- **Odometry.java** - Pose tracking and intelligent target prediction
- **generated/TunerConstants.java** - CTRE-generated swerve configuration

## Odometry.java - Target Prediction System

The most sophisticated part of the 2025 codebase. Contains two prediction algorithms:

### TargetPredictor (Advanced)

Motion-aware target selection with obstacle avoidance. Located at lines 301-551.

**Key Features:**

1. **Forced Selection Cone**
   - When robot is within 0.5m and inside a 42° cone in front of a reef side
   - That side is automatically locked in (prevents flickering)
   - See `getForcedConeTarget()` at line 458

2. **Ray-Casting Obstacle Avoidance**
   - Creates virtual "walls" for each reef side (0.875m perpendicular extension)
   - Checks if path to target intersects any reef walls
   - Filters out obstructed targets
   - See `isRayObstructed()` at line 494

   ```java
   // Each reef side creates a wall perpendicular to its face
   Translation2d wallLeft = new Translation2d(
     reefX + WALL_EXTENSION * cosLeft,
     reefY + WALL_EXTENSION * sinLeft);
   Translation2d wallRight = new Translation2d(
     reefX + WALL_EXTENSION * cosRight,
     reefY + WALL_EXTENSION * sinRight);
   ```

3. **Motion-Aware Cost Function**
   - Distance from element to robot's line of motion
   - Time cost to reach target
   - Energy cost to reorient (based on velocity direction change)
   - Prefers targets aligned with current motion

4. **Game State Awareness**
   - If holding coral → bias toward reef sides (0.8x cost multiplier)
   - If empty → bias toward coral stations (0.57x cost multiplier)
   - Uses `IntakeSubsystem.getInstance().coralDetected()`

5. **Confidence/Hysteresis System**
   - Prevents target flickering during navigation
   - Confidence builds when same target selected (increment: 0.15)
   - Confidence decays when different target is best (decrement: 0.09)
   - Target only switches when confidence drops below 0.3 threshold

### TargetPredictorSimple (Simplified)

Simplified version at lines 91-298. Distance-only with basic biasing.

**Differences from Advanced:**
- No velocity/motion awareness
- No energy cost calculation
- Pure Euclidean distance with game-state biasing
- Same obstruction and cone logic

### closestElement() Method

Simple utility function at line 586-611. **Not used by main prediction system.**

```java
public static GameElement closestElement(Pose2d robotPose)
```

Finds nearest GameElement by:
1. Euclidean distance (primary)
2. Angle difference (tiebreaker if equidistant)

**Note:** This doesn't consider obstacles or motion - use TargetPredictor for actual gameplay.

## Why 6 Reef Sides Matter for Prediction

The TargetPredictor system relies on the reef being split into 6 GameElements:

1. **Each side is a separate navigation target**
   - Robot can pick the best approach angle
   - Different sides may be obstructed while others are clear

2. **Ray-casting obstacle detection**
   - Each reef side creates a "wall" that can block paths
   - Prevents robot from trying to drive through the reef

3. **Cone-based forced selection**
   - When close to a specific side, locks onto that side
   - Prevents mid-approach target switching

## SwerveSubsystem.java

Extends CTRE's `TunerSwerveDrivetrain` with custom features:

### Choreo Path Following

```java
public void followPath(SwerveSample sample) {
  m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

  var pose = getPose();
  ChassisSpeeds speeds = new ChassisSpeeds(
    sample.vx + m_pathXController.calculate(pose.getX(), sample.x),
    sample.vy + m_pathYController.calculate(pose.getY(), sample.y),
    sample.omega + m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading)
  );

  setControl(m_pathApplyFieldSpeeds.withSpeeds(speeds));
}
```

**PID Controllers:**
- X/Y translation: P=1.5, I=0, D=0
- Rotation: P=10, I=0, D=0

### PathPlanner Integration

AutoBuilder configured with:
- Translation PID: P=7.51, I=0, D=0
- Rotation PID: P=1, I=0, D=0
- LocalADStar pathfinding algorithm

### Operator Perspective

Automatically sets forward direction based on alliance:
- Blue alliance: 0° (toward red wall)
- Red alliance: 180° (toward blue wall)

## MapleSimSwerveDrivetrain

Simulation support using MapleSimLib for testing without hardware.
