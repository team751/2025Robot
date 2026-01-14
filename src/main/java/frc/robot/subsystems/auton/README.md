# Autonomous Subsystem

## Overview

Manages autonomous routines using Choreo for trajectory generation and command composition for game actions.

## AutonSubsystem.java Architecture

### AutoFactory Setup

```java
autoFactory = new AutoFactory(
  swerveSubsystem::getPose,      // Get current pose
  swerveSubsystem::resetPose,    // Reset odometry to starting pose
  swerveSubsystem::followPath,   // Follow Choreo trajectory samples
  false,                         // Mirror (unused in this implementation)
  swerveSubsystem                // Subsystem requirement
);
```

ChoreoLib's `AutoFactory` bridges between:
- Choreo trajectory files (in `deploy/choreo/`)
- WPILib command-based framework
- Custom robot commands (scoring, intake, etc.)

### Autonomous Routine Pattern

Two approaches used in the codebase:

#### 1. Pure Trajectory Following (Generic)

```java
private AutoRoutine getAuton(String name) {
  AutoRoutine routine = autoFactory.newRoutine(name);
  List<Command> commandList = new ArrayList<>();

  // Load all trajectory segments for this auto
  int index = 0;
  while (true) {
    AutoTrajectory trajectory = routine.trajectory(name, index);
    if (trajectory.getFinalPose().equals(Optional.empty())) break;  // No more segments

    if (index == 0) commandList.add(trajectory.resetOdometry());  // Reset on first segment
    commandList.add(trajectory.cmd());                             // Follow this segment
    index++;
  }

  routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
  return routine;
}
```

**Usage:** Follows pre-planned paths from Choreo with no game actions.

#### 2. Game Action Integration (Specific Routines)

```java
private AutoRoutine getBlueRight() {
  AutoRoutine routine = autoFactory.newRoutine("blueright");
  List<Command> commandList = new ArrayList<>();

  // Navigate to reef and score
  commandList.add(new AssistCommand(GameElement.REEF_BLUE_2, GameElement.Branch.LEFT));
  commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

  // Navigate to coral station and intake
  commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null));
  commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(10)));

  // Navigate to different reef side and score again
  commandList.add(new AssistCommand(GameElement.REEF_BLUE_4, GameElement.Branch.RIGHT));
  commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

  routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
  return routine;
}
```

**Key Points:**

- **AssistCommand**: Dynamic pathfinding to GameElements using PathPlanner's LocalADStar
  - Creates waypoints for smooth approach
  - Handles branches for reef scoring positions
  - See `frc.robot.commands.AssistCommand` for implementation

- **Commands.race()**: Runs command with timeout
  - Allows game action to complete OR timeout expires
  - Prevents autonomous from hanging on failed actions

- **No pre-planned Choreo paths**: These routines use dynamic pathfinding only

### Autonomous Routines Defined

| Routine Name | Description |
|-------------|-------------|
| `hardcode auton` | Test routine cycling through multiple reef scores |
| `BlueRight` | Blue alliance right-side start |
| `BlueMid` | Blue alliance center start |
| `BlueLeft` | Blue alliance left-side start |
| `RedRight` | Red alliance right-side start |
| `RedMid` | Red alliance center start |
| `RedLeft` | Red alliance left-side start |
| `MoveAuton` | Simple forward movement (fallback) |

### AutoChooser

Routines exposed to driver station via SmartDashboard:
```java
autoChooser.addRoutine("BlueRight", () -> getBlueRight());
SmartDashboard.putData("Auto Chooser", autoChooser);
```

Driver selects autonomous mode on dashboard before match starts.

## Integration with Field Constants

Autonomous routines heavily use `GameElement` enum from `FieldConstants.java`:

```java
// Navigate to specific reef side and branch
new AssistCommand(GameElement.REEF_BLUE_2, GameElement.Branch.LEFT)

// Navigate to coral station (no branches)
new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null)
```

This abstracts field geometry from autonomous logic:
- Routines reference semantic locations (REEF_BLUE_2)
- Actual coordinates defined in FieldConstants
- Easy to update if field measurements change

## Choreo Trajectory Loading

For pre-planned paths (not used in main competition routines but shown in `getAuton()`):

1. Choreo trajectory file named `{name}.traj` in `deploy/choreo/`
2. AutoFactory loads trajectory by name
3. Trajectory can have multiple segments (split at stop points)
4. First segment resets odometry to starting pose

Example trajectory structure:
```
ExampleAuton.traj:
  - Segment 0: Start → Waypoint 1 (STOP)
  - Segment 1: Waypoint 1 → Waypoint 2 (STOP)
  - Segment 2: Waypoint 2 → End
```

Each segment is a separate command in the sequence.

## Command Sequencing

```java
routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
```

This pattern:
1. Waits for routine to be selected and enabled
2. Executes all commands in sequence
3. Each command must complete before next starts
4. Uses default command interruption behavior

## Testing Pattern

`MoveAuton` provides a simple "just move forward" routine:
```java
SwerveRequest swerveRequest = new SwerveRequest.FieldCentric()
  .withVelocityX(-2)
  .withVelocityY(0);

commandList.add(SwerveSubsystem.getInstance().applyRequest(() -> swerveRequest));
```

Useful for:
- Quick testing without complex paths
- Verifying basic mobility before match
- Emergency fallback if main autos fail
