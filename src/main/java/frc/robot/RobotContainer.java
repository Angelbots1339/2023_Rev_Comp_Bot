// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunWristAtVolts;
import frc.robot.commands.WristToPos;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.ElevatorSubsystem;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

  private GenericEntry wristPosEntry;
  private GenericEntry wristSpeedEntry;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new Drive(driveSubsystem,
                () -> MathUtil.applyDeadband(-driverController.getLeftY(), 0.06),
                () -> MathUtil.applyDeadband(-driverController.getLeftX(), 0.06),
                () -> MathUtil.applyDeadband(driverController.getRightX(), 0.06),
                true)
    );

    elevatorSubsystem.setDefaultCommand(new RunElevator(elevatorSubsystem, elevatorSpeedSupplier));
    
    ShuffleboardTab poseFinder = Shuffleboard.getTab("poseFinder");

    wristPosEntry = poseFinder.add("wrist Pos", 0).withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1)).getEntry();  

    wristSpeedEntry = poseFinder.add("wrist Speed Volts", 0).withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", -12, "max", 12)).getEntry();  
                  
                  }


  private final JoystickButton runIntake = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton runOuttake = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

  private final DoubleSupplier elevatorSpeedSupplier = () -> (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis()) / 3;

  private final JoystickButton wristToPos = new JoystickButton(driverController, XboxController.Button.kA.value);


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    runIntake.whileTrue(new RunIntake(intakeSubsystem, () -> 0.5));
    runOuttake.whileTrue(new RunIntake(intakeSubsystem, () -> -0.5));

    // wristToPos.whileTrue(new WristToPos(wristSubsystem, () -> wristPosEntry.getDouble(0)));

    // wristToPos.whileTrue(new RunWristAtVolts(wristSubsystem, () -> wristSpeedEntry.getDouble(0)));


    wristToPos.whileTrue(new InstantCommand(() -> driveSubsystem.resetYaw()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);
    
    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     driveSubsystem::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     driveSubsystem::setModuleStates,
    //     driveSubsystem);

    // // Reset odometry to the starting pose of the trajectory.
    // driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, false));
    return null;
  }
}