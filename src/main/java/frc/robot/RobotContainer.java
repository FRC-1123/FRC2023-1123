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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmLower;
import frc.robot.commands.ArmRaise;
import frc.robot.commands.ChargeStationBalance;
import frc.robot.commands.MiddleAutonomousDriving;
import frc.robot.commands.NewBalanceAlgorithm;
import frc.robot.commands.SetDrivetrainXForTime;
import frc.robot.commands.FlipIntake;
import frc.robot.commands.custom_wheel_angle;
import frc.robot.commands.goBackAnInch;
import frc.robot.commands.intakeInOrOut;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.computeTangentMove;
import frc.robot.commands.custom_wheel_angle;
import frc.robot.commands.readLimelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SensorSubsystem m_sensorSubsystem = new SensorSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  ShuffleboardTab teleopTab = Shuffleboard.getTab("teleopTab");
  ShuffleboardTab daArmTab = Shuffleboard.getTab("The ARM!");
  RunCommand fieldDriveOnOrOff;
  private final LimelightSubsystem limelight_test = new LimelightSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  GenericEntry upperArmVolt;
  GenericEntry lowerArmVolt;
  GenericEntry wristVolt;
  GenericEntry upperArmPos;
  GenericEntry lowerArmPos;
  GenericEntry wristPos;
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
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
    balance.setName("Balance");
    teleopTab.add("Charge Station Balancer", balance);
    
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

    GenericEntry intakeSetpoint = teleopTab.add("intake setpoint",0).getEntry();

    StartEndCommand intakeToggle = new StartEndCommand(()-> intakeSubsystem.setMotor(
        intakeSetpoint.getDouble(0)), ()-> intakeSubsystem.setStop(), intakeSubsystem);
    intakeToggle.setName("intake dashboard toggle");
    teleopTab.add("intake dashboard toggle", intakeToggle);

    // Command balanceAlgorithm = new NewBalanceAlgorithm(m_robotDrive, -1).andThen(new SetDrivetrainXForTime(m_robotDrive));
    // balanceAlgorithm.setName("new charge station balance");
    //teleopTab.add("new charge station balance", balanceAlgorithm);

    // Command balanceAlgorithmOtherWay = new NewBalanceAlgorithm(m_robotDrive, 1).andThen(new SetDrivetrainXForTime(m_robotDrive));
    // balanceAlgorithmOtherWay.setName("new charge station balance other way");
    // teleopTab.add("new charge station balance other way", balanceAlgorithmOtherWay);

    SequentialCommandGroup balanceAutonomous = new SequentialCommandGroup(
        new MiddleAutonomousDriving(m_robotDrive), new NewBalanceAlgorithm(m_robotDrive, 1), new SetDrivetrainXForTime(m_robotDrive));
    balanceAutonomous.setName("middle autonomous");
    teleopTab.add("Autonomus balance", balanceAutonomous);

    InstantCommand resetPoseToBeginning = new InstantCommand(
        ()-> m_robotDrive.resetOdometry(new Pose2d(0,0,new Rotation2d(Math.toRadians(180)))));
    resetPoseToBeginning.setName("reset pose to looking at driver");
    teleopTab.add("reset pose to looking at driver", resetPoseToBeginning);
    GenericEntry upperP = daArmTab.add("Upper P", .1).getEntry();
    GenericEntry upperI = daArmTab.add("Upper I", 0).getEntry();
    GenericEntry upperD = daArmTab.add("Upper D", 0).getEntry();
    GenericEntry lowerP = daArmTab.add("Lower P", .1).getEntry();
    GenericEntry lowerI = daArmTab.add("Lower I", 0).getEntry();
    GenericEntry lowerD = daArmTab.add("Lower D", 0).getEntry();
    GenericEntry wristP = daArmTab.add("Wrist P", 0).getEntry();
    GenericEntry wristI = daArmTab.add("Wrist I", 0).getEntry();
    GenericEntry wristD = daArmTab.add("Wrist D", 0).getEntry();
    InstantCommand setArmPID = new InstantCommand(()-> m_ArmSubsystem.setPid(
        upperP.getDouble(0), upperI.getDouble(0),
        upperD.getDouble(0), lowerP.getDouble(0),
        lowerI.getDouble(0), lowerD.getDouble(0),
        wristP.getDouble(0), wristI.getDouble(0),
        wristD.getDouble(0)));
    setArmPID.setName("Set Arm PID");
    daArmTab.add("PID arm setter", setArmPID);

    upperArmPos = daArmTab.add("Upper Arm Position", 0).getEntry();
    lowerArmPos = daArmTab.add("Lower Arm Position", 0).getEntry();
    wristPos = daArmTab.add("Wrist Position", 0).getEntry();
    InstantCommand setArmPos = new InstantCommand(()-> m_ArmSubsystem.setPosition(lowerArmPos.getDouble(0),
     upperArmPos.getDouble(0), wristPos.getDouble(0)));
    setArmPos.setName("Set Arm Position");
    daArmTab.add("Arm Position Setter", setArmPos);

    InstantCommand setLowerArm = new InstantCommand(()-> m_ArmSubsystem.setLowerPosition(lowerArmPos.getDouble(0)));
    setLowerArm.setName("set lower arm");
    daArmTab.add("set lower Arm position", setLowerArm);
    
    InstantCommand setUpperArm = new InstantCommand(()-> m_ArmSubsystem.setUpperPosition(upperArmPos.getDouble(0)));
    setUpperArm.setName("set upper arm");
    daArmTab.add("set upper Arm position", setUpperArm);

    InstantCommand setWrist = new InstantCommand(()-> m_ArmSubsystem.setWristPosition(wristPos.getDouble(0)));
    setWrist.setName("set wrist arm");
    daArmTab.add("set wrist position", setWrist);

    // GenericEntry stopUpperArm = daArmTab.add("Upper Arm Stop", 0).getEntry();
    // GenericEntry stopLowerArm = daArmTab.add("Lower Arm Stop", 0).getEntry();
    // GenericEntry stopWrist = daArmTab.add("Wrist Stop", 0).getEntry();
    InstantCommand stopArms = new InstantCommand(()-> m_ArmSubsystem.stopMotors());
    stopArms.setName("Stop Arms");
    daArmTab.add("Arm Stopper", stopArms);

    upperArmVolt = daArmTab.add("Upper Arm Voltage", 0).getEntry();
    lowerArmVolt = daArmTab.add("Lower Arm Voltage", 0).getEntry();
    wristVolt = daArmTab.add("Wrist Voltage", 0).getEntry();
    
    StartEndCommand armVolts = new StartEndCommand(()-> m_ArmSubsystem.setVoltage(lowerArmVolt.getDouble(0),
     upperArmVolt.getDouble(0), wristVolt.getDouble(0)), ()->m_ArmSubsystem.stopMotors());
    armVolts.setName("Set Voltage");
    daArmTab.add("Voltage Setter", armVolts);

    daArmTab.add("Flip intake up", new FlipIntake(m_ArmSubsystem, 10));
    daArmTab.add("flip intake down", new FlipIntake(m_ArmSubsystem, 165));

    InstantCommand resetArms = new InstantCommand(()->m_ArmSubsystem.resetArm());
    resetArms.setName("reset arms");
    daArmTab.add("reset arms", resetArms);

    InstantCommand setBrake = new InstantCommand(()-> m_ArmSubsystem.setBrake());
    setBrake.setName("set brake");
    daArmTab.add("set brake", setBrake);

    InstantCommand setCoast = new InstantCommand(()-> m_ArmSubsystem.setCoast());
    setCoast.setName("set coast");
    daArmTab.add("set coast", setCoast);

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
    autoChooserInit();
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
        //         Math.pow(MathUtil.applyDeadband(-testDriveController.getLeftY(), 0.06), 3)/2,
        //         Math.pow(MathUtil.applyDeadband(-testDriveController.getLeftX(), 0.06), 3)/2,
        //         Math.pow(MathUtil.applyDeadband(-testDriveController.getRightX(), 0.06), 3)/2,
        //         false);},
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
    new JoystickButton(driverJoystick, 5)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(driverJoystick, 1)
        .whileTrue(fieldDriveOnOrOff);


        
    //TODO change this to just press the button once instead of holding it
    new JoystickButton(driverJoystick, 10).onTrue(autoScoreCommandConeMedium);
    new JoystickButton(driverJoystick, 9).onTrue(autoScoreCommandCubeMedium);
    new JoystickButton(driverJoystick, 16).onTrue(autoScoreCommandConeTop);
    new JoystickButton(driverJoystick, 15).onTrue(autoScoreCommandCubeTop);



    StartEndCommand intakeOut = new StartEndCommand(() -> intakeSubsystem.setCone(), () -> intakeSubsystem.setStop(), intakeSubsystem);
    new JoystickButton(driverJoystick, 3).whileTrue(intakeOut);

    StartEndCommand intakeIn = new StartEndCommand(() -> intakeSubsystem.setCube(), () -> intakeSubsystem.setStop(), intakeSubsystem);
    new JoystickButton(driverJoystick, 4).whileTrue(intakeIn);
    
    // StartEndCommand lowerArmUp = new StartEndCommand(()-> m_ArmSubsystem.setLowerVoltage(-lowerArmVolt.getDouble(0)),
    //  ()-> m_ArmSubsystem.setLowerVoltage(0));
    // StartEndCommand lowerArmDown = new StartEndCommand(()-> m_ArmSubsystem.setLowerVoltage(lowerArmVolt.getDouble(0)),
    //  ()-> m_ArmSubsystem.setLowerVoltage(0));
    // StartEndCommand UpperArmUp = new StartEndCommand(()-> m_ArmSubsystem.setUpperVoltage(-upperArmVolt.getDouble(0)),
    //  ()-> m_ArmSubsystem.setUpperVoltage(0));
    // StartEndCommand UpperArmDown = new StartEndCommand(()-> m_ArmSubsystem.setUpperVoltage(upperArmVolt.getDouble(0)),
    //  ()-> m_ArmSubsystem.setUpperVoltage(0));
    // StartEndCommand wristUp = new StartEndCommand(()-> m_ArmSubsystem.setWristVoltage(-wristVolt.getDouble(0)),
    //  ()-> m_ArmSubsystem.setWristVoltage(0));
    // StartEndCommand wristDown = new StartEndCommand(()-> m_ArmSubsystem.setWristVoltage(wristVolt.getDouble(0)),
    //  ()-> m_ArmSubsystem.setWristVoltage(0));

    //  new JoystickButton(driverJoystick, 5)
    //  .whileTrue(lowerArmUp);
    // new JoystickButton(driverJoystick, 10)
    //  .whileTrue(lowerArmDown);
    // new JoystickButton(driverJoystick, 6)
    //  .whileTrue(UpperArmUp);
    // new JoystickButton(driverJoystick, 9)
    //  .whileTrue(UpperArmDown);
    // new JoystickButton(driverJoystick, 7)
    //  .whileTrue(wristUp);
    // new JoystickButton(driverJoystick, 8)
    //  .whileTrue(wristDown);


    // StartEndCommand setLowerPos = new StartEndCommand(()-> m_ArmSubsystem.setLowerPosition(lowerArmPos.getDouble(0)),
    //   ()-> m_ArmSubsystem.setLowerVoltage(0));
    // StartEndCommand setUpperPos = new StartEndCommand(()-> m_ArmSubsystem.setUpperPosition(upperArmPos.getDouble(0)),
    //   ()-> m_ArmSubsystem.setUpperVoltage(0));
    // StartEndCommand setWristPos = new StartEndCommand(()-> m_ArmSubsystem.setWristPosition(wristPos.getDouble(0)),
    //   ()-> m_ArmSubsystem.setWristVoltage(0));

    // new JoystickButton(driverJoystick, 13).whileTrue(setLowerPos);
    // new JoystickButton(driverJoystick, 12).whileTrue(setUpperPos);
    // new JoystickButton(driverJoystick, 11).whileTrue(setWristPos);
    InstantCommand stopArms = new InstantCommand(()-> m_ArmSubsystem.stopMotors());
    
    // new JoystickButton(driverJoystick, 1).whileTrue(stopArms);

    new JoystickButton(driverJoystick, 6).onTrue(new ArmRaise(m_ArmSubsystem, -173, 80, 205));
    new JoystickButton(driverJoystick, 7).onTrue(new ArmRaise(m_ArmSubsystem, -100, 36, 260, true));

    new JoystickButton(driverJoystick, 2).onTrue(new ArmLower(m_ArmSubsystem, 0, 0, 10));

    //change this
    //new JoystickButton(driverJoystick, 10).whileTrue(new FlipIntake(m_ArmSubsystem, 10));
    new JoystickButton(driverJoystick, 8).whileTrue(new FlipIntake(m_ArmSubsystem, 157));//165

    // new JoystickButton(driverJoystick, 8).whileTrue(new ArmLower(m_ArmSubsystem, 0, 0, 158));
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
// This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(m_chooser.getSelected(), new PathConstraints(2, 1));
System.out.println(m_chooser.getSelected());
// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("marker1", new PrintCommand("Passed marker 1"));

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    m_robotDrive::getPose, // Pose2d supplier
    m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(pathGroup);
return fullAuto;
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

  //The commands for the auto aim
  // for object_type: true = cone, false = cube
  SequentialCommandGroup autoScoreCommandConeMedium = new SequentialCommandGroup(
    new goBackAnInch(m_robotDrive, 3, 180),
    new readLimelight(limelight_test, true),
    new WaitCommand(0.05),
    new computeTangentMove(limelight_test, m_robotDrive, true, m_sensorSubsystem),
    new readLimelight(limelight_test, true),
    new WaitCommand(0.05),
    new computeTangentMove(limelight_test, m_robotDrive, true, m_sensorSubsystem),
    new goBackAnInch(m_robotDrive, 3, 0),
    new ArmRaise(m_ArmSubsystem, -100, 36, 260, true),
    new intakeInOrOut(intakeSubsystem, true, true),
    new ArmLower(m_ArmSubsystem, 0, 0, 10));
//-100, 36, 260
  SequentialCommandGroup autoScoreCommandConeTop = new SequentialCommandGroup(
    new goBackAnInch(m_robotDrive, 3, 180),
    new readLimelight(limelight_test, true),
    new WaitCommand(0.05),
    new computeTangentMove(limelight_test, m_robotDrive, true, m_sensorSubsystem),
    new readLimelight(limelight_test, true),
    new WaitCommand(0.05),
    new computeTangentMove(limelight_test, m_robotDrive, true, m_sensorSubsystem),
    new goBackAnInch(m_robotDrive, 3, 0),
    new ArmRaise(m_ArmSubsystem, -173, 80, 210),
    new intakeInOrOut(intakeSubsystem, true, true),
    new ArmLower(m_ArmSubsystem, 0, 0, 10));

  SequentialCommandGroup autoScoreCommandCubeMedium = new SequentialCommandGroup(
    new goBackAnInch(m_robotDrive, 3, 180),
    new readLimelight(limelight_test, false),
    new WaitCommand(0.05),
    new computeTangentMove(limelight_test, m_robotDrive, false),
    new readLimelight(limelight_test, false),
    new WaitCommand(0.05),
    new computeTangentMove(limelight_test, m_robotDrive, false),
    new goBackAnInch(m_robotDrive, 3, 0),
    new ArmRaise(m_ArmSubsystem, -100, 36, 260, true),
    new intakeInOrOut(intakeSubsystem, false, true),
    new ArmLower(m_ArmSubsystem, 0, 0, 10));

  SequentialCommandGroup autoScoreCommandCubeTop = new SequentialCommandGroup(
    new goBackAnInch(m_robotDrive, 3, 180),
    new readLimelight(limelight_test, false),
    new WaitCommand(0.05),
    new computeTangentMove(limelight_test, m_robotDrive, false),
    new readLimelight(limelight_test, false),
    new WaitCommand(0.05),
    new computeTangentMove(limelight_test, m_robotDrive, false),
    new goBackAnInch(m_robotDrive, 3, 0),
    new ArmRaise(m_ArmSubsystem, -173, 80, 205),
    new intakeInOrOut(intakeSubsystem, false, true),
    new ArmLower(m_ArmSubsystem, 0, 0, 10));

  private static final String kDefaultAuto = "big blue safe (good)";
  private static final String kCustomAuto1 = "left blue 2 piece (good)";
  private static final String kCustomAuto2 = "left blue escape (good)";
  private static final String kCustomAuto3 = "middle blue 2 peice (good)";
  private static final String kCustomAuto4 = "right 2 peice (good)";
  private static final String kCustomAuto5 = "right blue escape (good)";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public void autoChooserInit() {
    m_chooser.setDefaultOption("Big Blue Safe", kDefaultAuto);
    m_chooser.addOption("Left Blue 2 Piece", kCustomAuto1);
    m_chooser.addOption("Left Blue Escape", kCustomAuto2);
    m_chooser.addOption("Middle Blue 2 Piece", kCustomAuto3);
    m_chooser.addOption("Right 2 Piece", kCustomAuto4);
    m_chooser.addOption("Right Blue Escape", kCustomAuto5);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  
}
 