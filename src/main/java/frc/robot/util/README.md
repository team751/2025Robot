# Field Constants and Game Elements

## Overview

This directory contains `FieldConstants.java`, which defines all fixed field geometry for the 2025 FRC game (REEFSCAPE).

## GameElement Enum - Important Distinction

**GameElement represents FIELD COMPONENTS, not game pieces.**

- ✅ Field components with fixed positions: Reefs, Coral Stations, Cages, Processors
- ❌ NOT game pieces: Coral and Algae (these move during the match)

### Why This Matters

The robot needs known `Pose2d` coordinates for autonomous navigation. Only fixed field features have predictable positions. Game pieces move around and are tracked differently (via vision, sensors, etc.).

## Reef Hexagon Architecture

Each hexagonal reef is modeled as **6 separate GameElement entries** (one per side):

```
Red Reef:  REEF_RED_1, REEF_RED_2, REEF_RED_3, REEF_RED_4, REEF_RED_5, REEF_RED_6
Blue Reef: REEF_BLUE_1, REEF_BLUE_2, REEF_BLUE_3, REEF_BLUE_4, REEF_BLUE_5, REEF_BLUE_6
```

### Why 6 Separate Elements?

Breaking the hexagon into 6 sides enables:

1. **Intelligent approach selection** - Robot can choose the closest/best side to approach
2. **Obstacle avoidance** - Ray-casting checks if other reef sides block the path
3. **Angle-aware navigation** - Each side has its own heading (0°, 60°, 120°, 180°, 240°, 300°)
4. **Precise scoring** - Each side has 3 scoring positions (left/mid/right branches)

### Reef Side Structure

Each reef side has:
- **Center position** - AprilTag location on that side
- **Heading** - Direction perpendicular to that side
- **Branches** - Three scoring positions:
  - Left branch: Offset 90° left from heading
  - Mid branch: Directly at center
  - Right branch: Offset 90° right from heading

Example from `FieldConstants.java`:
```java
REEF_RED_1(
  new Branches(
    offsetByAngle(new Pose2d(13.8784, 4.0386, Rotation2d.fromDegrees(0)), CORAL_OFFSET, -90),  // left
    new Pose2d(13.8784, 4.0386, Rotation2d.fromDegrees(0)),                                      // mid
    offsetByAngle(new Pose2d(13.8784, 4.0386, Rotation2d.fromDegrees(0)), CORAL_OFFSET, +90)   // right
  ),
  false  // isBlue = false (red alliance)
),
```

## Other Field Elements

### Coral Stations
Pickup locations where human players feed coral to robots:
- `CORAL_STATION_RED_1`, `CORAL_STATION_RED_2`
- `CORAL_STATION_BLUE_1`, `CORAL_STATION_BLUE_2`

Single `Pose2d` each (no branches).

### Cages and Processors
Barge-related game elements:
- `CAGE_RED_1/2/3`, `CAGE_BLUE_1/2/3`
- `PROCESSOR_RED`, `PROCESSOR_BLUE`

Marked with `ignoreByTargetPredictor = true` to exclude from automatic target selection.

## Utility Methods

### `offsetByAngle()`
Calculates a perpendicular offset from a pose:
```java
public static Pose2d offsetByAngle(Pose2d center, double offsetMeters, double angleOffsetDegrees)
```

Used to create branch positions offset from the reef center.

### `getColor()`
Filters elements by alliance:
```java
public static List<GameElement> getColor(boolean isBlue)
```

Returns only red or blue alliance elements for autonomous routines.

### `getPoseWithOffset()`
Calculates approach position in front of an element:
```java
public static Pose2d getPoseWithOffset(GameElement element, double offset)
```

Used extensively by navigation commands to position robot at scoring distance.

## Coordinate System

- **Origin**: Blue alliance lower-left corner
- **Units**: Meters
- **X-axis**: Increases toward red alliance
- **Y-axis**: Increases toward drivers' left
- **Rotation**: Counter-clockwise positive (standard mathematical convention)

## Usage in Code

### Autonomous Routines
```java
commandList.add(new AssistCommand(GameElement.REEF_BLUE_2, GameElement.Branch.LEFT));
commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null));
```

### Target Prediction
See `Odometry.java` for the sophisticated target prediction system that uses these GameElements with ray-casting obstacle avoidance and motion-aware scoring.
