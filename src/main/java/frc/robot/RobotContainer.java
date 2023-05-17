package frc.robot;

import frc.robot.Constants.DriverControlConsts;
import frc.robot.Constants.SwerveConsts;
import frc.robot.commands.AutonomousCommands.*;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.LED_Commands.*;
import frc.robot.subsystems.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  public static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static PivotSubsystem pivotSubsystem = new PivotSubsystem();
  public static ClawSubsystem clawSubsystem = new ClawSubsystem();
  public static Lights lights = new Lights();

  private XboxController xbox = new XboxController(DriverControlConsts.XBOX_CONTROLLER_PORT);
  private Joystick joystick = new Joystick(DriverControlConsts.JOYSTICK_PORT);

  //AUTONOMOUS CHOICES 
  private Command highMobility = new HighMobility(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command highBal = new HighBal(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command high = new High(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command doNothing = new DoNothing();
  private Command redHighBalEnc = new RedHighBalEnc(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem); 
  private Command blueHighBalEnc = new BlueHighBalEnc(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command mixedBalance = new MixedBalance(swerveSubsystem);
  private Command swerveLock = new Lock(swerveSubsystem);
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new FieldOriented(swerveSubsystem,
        () -> xbox.getLeftY() * 0.95,
        () -> xbox.getLeftX() * 0.95,
        () -> -xbox.getRightX() * 0.95));

    selectAuto();
    configureBindings();
  }
  
  private void configureBindings() {

    /* SWERVE */

    new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem));
    // new JoystickButton(xbox, 4).toggleOnTrue(new Endgame(swerveSubsystem, () -> xbox.getLeftY()));
    new JoystickButton(xbox, 4).toggleOnTrue(new TankEndgame(swerveSubsystem, () -> xbox.getLeftY(), () -> -xbox.getRightY()));

    // FOR TESTING
    new JoystickButton(xbox, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));
    new JoystickButton(xbox, 3).onTrue(new LandingGearIn(swerveSubsystem));

    /* CLAW */
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));

    new JoystickButton(joystick, 8).onTrue(new Go90Clockwise(clawSubsystem));
    new JoystickButton(joystick, 10).onTrue(new ToStartingPosition(clawSubsystem));
    new JoystickButton(joystick, 12).onTrue(new Go90Counterclockwise(clawSubsystem));

    new JoystickButton(joystick, 2).whileTrue(new ManualClaw(clawSubsystem, () -> joystick.getX()));

    /* PIVOT */
    new JoystickButton(joystick, 11).onTrue(new LowPickUp(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(joystick, 9).onTrue(new ParallelCommandGroup(new PivotMiddleCommand(pivotSubsystem), new MidPosition(elevatorSubsystem)));
    new JoystickButton(joystick, 7).onTrue(new TopNode(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(joystick, 5).onTrue(Tucked.getCommand(pivotSubsystem, elevatorSubsystem, clawSubsystem));
    new JoystickButton(joystick, 1).whileTrue(new PivotJoystickCommand(pivotSubsystem, () -> -joystick.getY()));

    /* ELEVATOR */
    new POVButton(joystick, 0).whileTrue(new ManualElevatorDrive(elevatorSubsystem, 0.75));
    new POVButton(joystick, 180).whileTrue(new ManualElevatorDrive(elevatorSubsystem, -0.75));

    // new JoystickButton(joystick, 3).onTrue(new LowPosition(elevatorSubsystem));

    /* LIGHTS */
    new JoystickButton(joystick, 6).toggleOnTrue(new Yellow(lights));
    new JoystickButton(joystick, 4).toggleOnTrue(new Violet(lights));

  }
  

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getSwerveLock(){
    return swerveLock;
  }

  public void selectAuto() {
    autoChooser.setDefaultOption("Do Nothing", doNothing);
    autoChooser.addOption("High Mobility", highMobility);
    autoChooser.addOption("High Balance", highBal);
    autoChooser.addOption("Red High Balance", redHighBalEnc);
    autoChooser.addOption("Blue High Balance", blueHighBalEnc);
    autoChooser.addOption("High ONLY", high);
    autoChooser.addOption("Mixed Balance ONLY", mixedBalance);

    
    autoChooser.addOption("TEST", test());

    SmartDashboard.putData(autoChooser);
  }
  
  public Command test(){
    SmartDashboard.putString("Current Command", "Running");
    // Trajectory Settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(SwerveConsts.MAX_SPEED, 1)
        .setKinematics(SwerveConsts.DRIVE_KINEMATICS);

    // Trajectory Generator
    // (initial position, interior waypoints, ending position, trajectory configuration)
    // interior point = points to go through
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
          new Translation2d(0.01, 0)
        ),
        new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig
    );

    // Correct errors in trajectory
    PIDController xController = new PIDController(0.01, 0, 0);
    PIDController yController = new PIDController(0.01, 0, 0);
    ProfiledPIDController angleController = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(SwerveConsts.MAX_ROTATION, 1)); // FIXME
    // like PIDController but adds limit on maximum speed and acceleration
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Command to follow trajectory
    // :: => shorthand for lambda for calling specific method (method reference operator)
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      swerveSubsystem::getPose,
      SwerveConsts.DRIVE_KINEMATICS,
      xController, yController, angleController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem
      );

    return new SequentialCommandGroup(
      new InstantCommand( () -> swerveSubsystem.resetOdometry(trajectory.getInitialPose()) ),
      swerveControllerCommand,
      new InstantCommand( () -> swerveSubsystem.stopModules())
    );

  }


}
