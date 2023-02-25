// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAimLimelight;
import frc.robot.commands.ChargeStationBalance;
import frc.robot.commands.custom_wheel_angle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  ShuffleboardTab teleopTab = Shuffleboard.getTab("teleopTab");
  RunCommand fieldDriveOnOrOff;
  private final LimelightSubsystem limelight_test = new LimelightSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private void shuffleboardContainment()
  {
   fieldDriveOnOrOff =  new RunCommand(
            () -> {double motorSpeed = -driverJoystick.getThrottle();  // Get the raw value
                motorSpeed = motorSpeed + 1;                                 // Range of 0-2
                motorSpeed = motorSpeed / 2;    
                m_robotDrive.drive(
                MathUtil.applyDeadband(-driverJoystick.getY(), 0.06)*motorSpeed,
                MathUtil.applyDeadband(-driverJoystick.getX(), 0.06)*motorSpeed,
                MathUtil.applyDeadband(-driverJoystick.getZ(), 0.1)*motorSpeed,
                false);},
            m_robotDrive);
    fieldDriveOnOrOff.setName("Enable robot orientation.");
    teleopTab.add("Drive Orientation",fieldDriveOnOrOff);

    InstantCommand gyroResetInstantCommand = new InstantCommand(()->m_robotDrive.zeroHeading());
    gyroResetInstantCommand.setName("Reset Gyro");
    teleopTab.add("Gyro Reset", gyroResetInstantCommand);

    InstantCommand encoderReset = new InstantCommand(() -> m_robotDrive.resetEncoders());
    encoderReset.setName("Reset Encoders");
    teleopTab.add("Encoders", encoderReset);

    GenericEntry fRightAngle = teleopTab.add("Front Right Angle", 0).getEntry();
    GenericEntry rRightAngle = teleopTab.add("Rear Right Angle", 0).getEntry();
    GenericEntry fLeftAngle = teleopTab.add("Front Left Angle", 0).getEntry();
    GenericEntry rLeftAngle = teleopTab.add("Rear Left Angle", 0).getEntry();
    custom_wheel_angle theCustomWheelAngleCommand = new custom_wheel_angle(m_robotDrive, fRightAngle, rRightAngle, fLeftAngle, rLeftAngle);
    teleopTab.add("The Weel Angel", theCustomWheelAngleCommand);

    //balances on the charge station
    ChargeStationBalance balance = new ChargeStationBalance(m_robotDrive);
    balance.setName("The name! v2");
    teleopTab.add("Totally 100% abosultely balanced on charge station", balance);
    
    //gives you the X, Y, and rotation angle from the getPose() command (Dosent display it)
    GenericEntry movementX = teleopTab.add("X Position", 0).getEntry();
    GenericEntry movementY = teleopTab.add("Y Position", 0).getEntry();
    GenericEntry positionAngle = teleopTab.add("Position Angle", 0).getEntry();
    
    //moves one meter forward using the code above
    InstantCommand setMovement = new InstantCommand(()-> generateSwerveCommand(m_robotDrive.getPose(),
    new Pose2d(movementX.getDouble(0) + m_robotDrive.getPose().getX(), movementY.getDouble(0)
     + m_robotDrive.getPose().getY(), new Rotation2d(positionAngle.getDouble(0)))).schedule());
    setMovement.setName("meater mover eine");
    teleopTab.add("Met er move er", setMovement);
    
    //absolute move back
    InstantCommand goToPosition = new InstantCommand(()-> generateSwerveCommand(m_robotDrive.getPose(),
    new Pose2d(movementX.getDouble(0), movementY.getDouble(0),
    new Rotation2d(positionAngle.getDouble(0)))).schedule());
    goToPosition.setName("the button that moves-back-inator");
    teleopTab.add("move-back-inator", goToPosition);

    InstantCommand poseResetterCommand = new InstantCommand(()-> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));
    poseResetterCommand.setName("Reset pose");
    teleopTab.add("Pose resetter", poseResetterCommand);

    AutoAimLimelight autoAimWithLimelight = new AutoAimLimelight(m_robotDrive, limelight_test);
    teleopTab.add("Auto Aim Cone", autoAimWithLimelight);

    GenericEntry intakeSetpoint = teleopTab.add("intake setpoint",0).getEntry();

    StartEndCommand intakeToggle = new StartEndCommand(()-> intakeSubsystem.setMotor(
        intakeSetpoint.getDouble(0)), ()-> intakeSubsystem.setStop(), intakeSubsystem);
    intakeToggle.setName("intake dashboard toggle");
    teleopTab.add("intake dashboard toggle", intakeToggle);

}

  // The driver's controller
  Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  /**q
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_robotDrive.zeroHeading();
    shuffleboardContainment();
    configureButtonBindings();

    //Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {double motorSpeed = -driverJoystick.getThrottle();  // Get the raw value
                motorSpeed = motorSpeed + 1;                                 // Range of 0-2
                motorSpeed = motorSpeed / 2;    
                m_robotDrive.drive(
                MathUtil.applyDeadband(-driverJoystick.getY(), 0.06)*motorSpeed,
                MathUtil.applyDeadband(-driverJoystick.getX(), 0.06)*motorSpeed,
                MathUtil.applyDeadband(-driverJoystick.getZ(), 0.1)*motorSpeed,
                true);},
            m_robotDrive));
        // XboxController testDriveController = new XboxController(1);
        // m_robotDrive.setDefaultCommand(
        //     new RunCommand(
        //     () -> {   
        //         m_robotDrive.drive(
        //         Math.pow(MathUtil.applyDeadband(-testDriveController.getLeftX(), 0.06), 2),
        //         Math.pow(MathUtil.applyDeadband(-testDriveController.getLeftY(), 0.06), 2),
        //         Math.pow(MathUtil.applyDeadband(-testDriveController.getRightX(), 0.06), 2),
        //         true);},
        //     m_robotDrive));
        }

  

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
    new JoystickButton(driverJoystick, 4)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(driverJoystick, 1)
        .whileTrue(fieldDriveOnOrOff);

    new JoystickButton(driverJoystick, 9).whileTrue(autoScoreCommand);

    StartEndCommand intakeOut = new StartEndCommand(() -> intakeSubsystem.setCone(), () -> intakeSubsystem.setStop(), intakeSubsystem);
    new JoystickButton(driverJoystick, 4).whileTrue(intakeOut);

    StartEndCommand intakeIn = new StartEndCommand(() -> intakeSubsystem.setCube(), () -> intakeSubsystem.setStop(), intakeSubsystem);
    new JoystickButton(driverJoystick, 3).whileTrue(intakeIn);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SequentialCommandGroup autonomousCommand = new SequentialCommandGroup(
        generateSwerveCommand(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), new Pose2d(-4, 0, new Rotation2d(Math.toRadians(0)))), 
        new InstantCommand(()-> m_robotDrive.drive(0, 0, 0, false)));

    //Simple autonomus, top right on blue facing scoring tables
    //     SequentialCommandGroup longAuto = new SequentialCommandGroup(
    // scoreGamePeiceCommand(), //score the held game Peice (it just waits for now)
    // generateSwerveCommand(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(-4.34, 0, new Rotation2d(0))),  //this command moves the robot backwards 4.34 meters
    // generateSwerveCommand(new Pose2d(-4.34, 0, new Rotation2d(0)), new Pose2d(-4.34, 0, new Rotation2d(180))),  //this command turns the robot 180 degrees toward the GP
    // generateSwerveCommand(new Pose2d(-4.34, 0, new Rotation2d(180)), new Pose2d(-4.79, 0, new Rotation2d(180))),  //this command moves the robot 18 inches over the GP
    // generateSwerveCommand(new Pose2d(-4.79, 0, new Rotation2d(180)), new Pose2d(-4.79, 0, new Rotation2d(0))),  //this command turns the robot 180 degrees back toward the scoring wall
    // generateSwerveCommand(new Pose2d(-4.79, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0))),  //this command returns the robot 4.79 meters back to the scoring wall
    
    //score the collected game Peice
    // new InstantCommand(()-> m_robotDrive.drive(0, 0, 0, false)),
    // scoreGamePeiceCommand());
    
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));   
    
    // Run path following command, then stop at the end.
    return autonomousCommand;
  


}

  public Command scoreGamePeiceCommand(){
    return new WaitCommand(3.1);
  }
  
  private Command generateSwerveCommand(Pose2d startPosition, Pose2d endPosition){
    System.out.println(startPosition.getRotation().getDegrees() + " stuff " + endPosition.getRotation().getDegrees());
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            startPosition,
            List.of(),
            endPosition,
            config);
    
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        return swerveControllerCommand;
  }

  SequentialCommandGroup autoScoreCommand = new SequentialCommandGroup(/*read limelight, compute move, move, score */);

}
