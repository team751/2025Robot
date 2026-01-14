package frc.robot.subsystems.auton;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AssistCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.FieldConstants.GameElement;
// import the wpilib waitcommand
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Manages autonomous routines using Choreo for trajectory generation.
 *
 * <p>TWO AUTONOMOUS PATTERNS USED:
 *
 * <p>1. PURE TRAJECTORY FOLLOWING (see getAuton()):
 * - Loads pre-planned Choreo paths from deploy/choreo/*.traj
 * - Splits multi-segment paths at stop points
 * - Resets odometry at start
 * - No game-specific actions (testing/reference only)
 *
 * <p>2. DYNAMIC PATHFINDING (see getBlueRight(), etc.):
 * - Uses AssistCommand for on-the-fly navigation to GameElements
 * - PathPlanner's LocalADStar generates paths at runtime
 * - Integrates game actions (score, intake) with navigation
 * - ACTUALLY USED IN COMPETITION
 *
 * <p>Competition routines combine:
 * - AssistCommand(GameElement, Branch) → Navigate to field location
 * - ScoreCommand/IntakeCommand → Game-specific mechanism actions
 * - Commands.race() → Timeout wrapper for safety
 *
 * <p>The GameElement enum (6 reef sides, coral stations) provides semantic targets
 * without hard-coding coordinates in autonomous routines.
 */
public class AutonSubsystem {
private final AutoChooser autoChooser = new AutoChooser();

private final AutoFactory autoFactory;

private final Superstructure superstructure;
private final SwerveSubsystem swerveSubsystem;

private static AutonSubsystem instance;

private AutonSubsystem() {
	superstructure = Superstructure.getInstance();
	swerveSubsystem = SwerveSubsystem.getInstance();

	autoFactory =
		new AutoFactory(
			swerveSubsystem::getPose,
			swerveSubsystem::resetPose,
			swerveSubsystem::followPath,
			false,
			swerveSubsystem);

	autoChooser.addRoutine("hardcode auton", () -> getBadAuton());
	autoChooser.addRoutine("BlueRight", () -> getBlueRight());
	autoChooser.addRoutine("BlueMid", () -> getBlueMid());
	autoChooser.addRoutine("BlueLeft", () -> getBlueLeft());
	autoChooser.addRoutine("RedRight", () -> getRedRight());
	autoChooser.addRoutine("RedMid", () -> getRedMid());
	autoChooser.addRoutine("RedLeft", () -> getRedLeft());
	autoChooser.addRoutine("MoveAuton", () -> getMoveAuton());
	SmartDashboard.putData("Auto Chooser", autoChooser);
}

public static AutonSubsystem getInstance() {
	if (instance == null) instance = new AutonSubsystem();
	return instance;
}

public Command getSelectedAuton() {
	return autoChooser.selectedCommand();
}

/**
 * Generic trajectory follower for pre-planned Choreo paths.
 *
 * <p>Loads a Choreo trajectory by name from deploy/choreo/{name}.traj
 * Handles multi-segment paths (split at stop points in Choreo).
 *
 * <p>NOTE: Not used in competition routines - competition uses dynamic pathfinding
 * with AssistCommand instead. This is kept for testing/reference.
 *
 * @param name Name of the Choreo trajectory file (without .traj extension)
 * @return AutoRoutine that follows all segments of the path
 */
private AutoRoutine getAuton(String name) {
	AutoRoutine routine = autoFactory.newRoutine(name);
	List<Command> commandList = new ArrayList<>();

	// Load all trajectory segments (Choreo splits paths at stop points)
	int index = 0;
	while (true) {
	AutoTrajectory trajectory = routine.trajectory(name, index);
	if (trajectory.getFinalPose().equals(Optional.empty())) break;  // No more segments
	if (index == 0) commandList.add(trajectory.resetOdometry());  // Reset on first segment only
	commandList.add(trajectory.cmd());  // Follow this segment

	// commandList.add(new AssistCommand(false, true));
	// commandList.add(new WaitCommand(1));
	index++;
	}

	// Register the full sequence of commands to run when routine is active
	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

/**
 * Blue alliance autonomous starting from right side of field.
 *
 * <p>PATTERN: Navigate → Score → Navigate → Intake → Navigate → Score
 *
 * <p>Uses dynamic pathfinding (AssistCommand) rather than pre-planned Choreo paths.
 * AssistCommand generates paths on-the-fly using PathPlanner's LocalADStar algorithm.
 *
 * <p>Commands.race() provides timeout safety - game action completes OR timeout expires.
 * Prevents autonomous from hanging if a mechanism fails.
 *
 * <p>Note: Reef sides (REEF_BLUE_2, REEF_BLUE_4) are separate GameElements from the
 * hexagonal reef structure. Robot picks specific side to approach based on starting position.
 */
private AutoRoutine getBlueRight() {
	AutoRoutine routine = autoFactory.newRoutine("blueright");
	List<Command> commandList = new ArrayList<>();

	// Score preloaded coral on reef
	commandList.add(new AssistCommand(GameElement.REEF_BLUE_2, GameElement.Branch.LEFT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

	// Pick up coral from human player station
	commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(10)));
	// commandList.add(new WaitCommand(5));

	commandList.add(new AssistCommand(GameElement.REEF_BLUE_4, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

	// commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null));
	//        commandList.add(Commands.race(
	//                new IntakeCommand(),
	//                new WaitCommand(10)
	//        ));

	// commandList.add(new AssistCommand(GameElement.REEF_RED_1, GameElement.Branch.LEFT));
	// Register the full sequence of commands to run when routine is active
	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

private AutoRoutine getBlueMid() {
	AutoRoutine routine = autoFactory.newRoutine("bluemid");
	List<Command> commandList = new ArrayList<>();

	commandList.add(new AssistCommand(GameElement.REEF_BLUE_1, GameElement.Branch.LEFT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

	commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_1, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(10)));

	commandList.add(new AssistCommand(GameElement.REEF_BLUE_6, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));
	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

private AutoRoutine getBlueLeft() {
	AutoRoutine routine = autoFactory.newRoutine("blueleft");
	List<Command> commandList = new ArrayList<>();

	commandList.add(new AssistCommand(GameElement.REEF_BLUE_3, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

	commandList.add(new AssistCommand(GameElement.CORAL_STATION_BLUE_2, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(10)));

	commandList.add(new AssistCommand(GameElement.REEF_BLUE_5, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));
	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

private AutoRoutine getRedRight() {
	AutoRoutine routine = autoFactory.newRoutine("redright");
	List<Command> commandList = new ArrayList<>();

	commandList.add(new AssistCommand(GameElement.REEF_RED_5, GameElement.Branch.LEFT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

	commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_2, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(10)));

	commandList.add(new AssistCommand(GameElement.REEF_RED_3, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));
	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

private AutoRoutine getRedMid() {
	AutoRoutine routine = autoFactory.newRoutine("redmid");
	List<Command> commandList = new ArrayList<>();

	commandList.add(new AssistCommand(GameElement.REEF_RED_6, GameElement.Branch.LEFT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

	commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_2, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(10)));

	commandList.add(new AssistCommand(GameElement.REEF_RED_1, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));
	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

private AutoRoutine getRedLeft() {
	AutoRoutine routine = autoFactory.newRoutine("redleft");
	List<Command> commandList = new ArrayList<>();

	commandList.add(new AssistCommand(GameElement.REEF_RED_4, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));

	commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_1, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(10)));

	commandList.add(new AssistCommand(GameElement.REEF_RED_2, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(5)));
	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

private AutoRoutine getMoveAuton() {
	AutoRoutine routine = autoFactory.newRoutine("MoveAuton");
	List<Command> commandList = new ArrayList<>();

	SwerveRequest swerveRequest =
		new SwerveRequest.FieldCentric().withVelocityX(-2).withVelocityY(0);

	commandList.add(
		new InstantCommand(
			() ->
				SwerveSubsystem.getInstance()
					.resetRotation(SwerveSubsystem.getInstance().getOperatorForwardDirection())));
	commandList.add(SwerveSubsystem.getInstance().applyRequest(() -> swerveRequest));

	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

private AutoRoutine getBadAuton() {
	AutoRoutine routine = autoFactory.newRoutine("imbad");
	List<Command> commandList = new ArrayList<>();

	commandList.add(new AssistCommand(GameElement.REEF_RED_6, GameElement.Branch.LEFT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(1)));

	commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_1, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(1)));

	commandList.add(new AssistCommand(GameElement.REEF_RED_3, GameElement.Branch.RIGHT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(1)));

	commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_2, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(1)));

	commandList.add(new AssistCommand(GameElement.REEF_RED_2, GameElement.Branch.LEFT));
	commandList.add(Commands.race(new ScoreCommand(ScoreCommand.Level.L4), new WaitCommand(1)));

	commandList.add(new AssistCommand(GameElement.CORAL_STATION_RED_2, null));
	commandList.add(Commands.race(new IntakeCommand(), new WaitCommand(1)));

	commandList.add(new AssistCommand(GameElement.REEF_RED_1, GameElement.Branch.LEFT));

	// Register the full sequence of commands to run when routine is active
	routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
	return routine;
}

private AutoRoutine getExampleAuton() {
	AutoRoutine routine = autoFactory.newRoutine("ExampleAuton");
	AutoTrajectory trajectory = routine.trajectory("ExampleAuton");

	routine.active().onTrue(trajectory.resetOdometry().andThen(trajectory.cmd()));
	return routine;
}
}
