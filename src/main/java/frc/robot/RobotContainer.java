// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmLower;
import frc.robot.commands.ArmRaise;
import frc.robot.commands.ArmRaiseForAuto;
import frc.robot.commands.ArmRaisePrepare;
import frc.robot.commands.ArmRaiseScoringCube;
import frc.robot.commands.ArmRaiseSubstation;
import frc.robot.commands.AutoBalanceHelper;
import frc.robot.commands.AutoIntakeInOrOut;
import frc.robot.commands.DriveForTime;
import frc.robot.commands.DriveForTimeHoldRotation;
import frc.robot.commands.ExAutoAim;
import frc.robot.commands.MiddleAutonomousDriving;
import frc.robot.commands.MotorDiagnostic;
import frc.robot.commands.MoveASmallDistance;
import frc.robot.commands.MoveASmallDistancePid;
import frc.robot.commands.MoveATinyDistancePid;
import frc.robot.commands.MoveArmToFeeder;
import frc.robot.commands.MoveUntilCone;
import frc.robot.commands.MoveUntilCube;
import frc.robot.commands.MoveUntilCubeSetSpeed;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.RotateToAnglePID;
import frc.robot.commands.RotateToAnglePIDAgressive;
import frc.robot.commands.RotateToAngleTest;
import frc.robot.commands.RunIntakeUntilStall;
import frc.robot.commands.SetDrivetrainXForTime;
import frc.robot.commands.ShootCubeSlow;
import frc.robot.commands.SpeedTest;
import frc.robot.commands.SpitOutSlowAuto;
import frc.robot.commands.StopUntilCone;
import frc.robot.commands.TestingAutoBalance;
import frc.robot.commands.custom_wheel_angleInput;
import frc.robot.commands.custom_wheel_angleInputFast;
import frc.robot.commands.FlipIntake;
import frc.robot.commands.GoToPosition;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.intakeInOrOut;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.readLimelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MAXSwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  public XboxController copilotController = new XboxController(OIConstants.kCopilotConrollerPort);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  ShuffleboardTab teleopTab = Shuffleboard.getTab("teleopTab");
  ShuffleboardTab daArmTab = Shuffleboard.getTab("The ARM!");
  ShuffleboardTab diagnosticTab = Shuffleboard.getTab("Diagnostics");
  RunCommand fieldDriveOnOrOff;
  private final LimelightSubsystem limelight_test = new LimelightSubsystem();
  private final SensorSubsystem m_sensorSubsystem = new SensorSubsystem(limelight_test, copilotController);
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

    InstantCommand poseResetterCommand = new InstantCommand(()-> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));
    poseResetterCommand.setName("Reset pose");
    teleopTab.add("Pose resetter", poseResetterCommand);

    GenericEntry intakeSetpoint = teleopTab.add("intake setpoint",0).getEntry();

    StartEndCommand intakeToggle = new StartEndCommand(()-> intakeSubsystem.setMotor(
        intakeSetpoint.getDouble(0)), ()-> intakeSubsystem.setStop(), intakeSubsystem);
    intakeToggle.setName("intake dashboard toggle");
    teleopTab.add("intake dashboard toggle", intakeToggle);

    InstantCommand resetPoseToBeginning = new InstantCommand(
        ()-> m_robotDrive.resetOdometry(new Pose2d(0,0,new Rotation2d(Math.toRadians(180)))));
    resetPoseToBeginning.setName("reset pose to looking at driver");
    teleopTab.add("reset pose to looking at driver", resetPoseToBeginning);

    // GenericEntry upperP = daArmTab.add("Upper P", .1).getEntry();
    // GenericEntry upperI = daArmTab.add("Upper I", 0).getEntry();
    // GenericEntry upperD = daArmTab.add("Upper D", 0).getEntry();
    // GenericEntry lowerP = daArmTab.add("Lower P", .1).getEntry();
    // GenericEntry lowerI = daArmTab.add("Lower I", 0).getEntry();
    // GenericEntry lowerD = daArmTab.add("Lower D", 0).getEntry();
    // GenericEntry wristP = daArmTab.add("Wrist P", 0).getEntry();
    // GenericEntry wristI = daArmTab.add("Wrist I", 0).getEntry();
    // GenericEntry wristD = daArmTab.add("Wrist D", 0).getEntry();
    // InstantCommand setArmPID = new InstantCommand(()-> m_ArmSubsystem.setPid(
    //     upperP.getDouble(0), upperI.getDouble(0),
    //     upperD.getDouble(0), lowerP.getDouble(0),
    //     lowerI.getDouble(0), lowerD.getDouble(0),
    //     wristP.getDouble(0), wristI.getDouble(0),
    //     wristD.getDouble(0)));
    // setArmPID.setName("Set Arm PID");
    // daArmTab.add("PID arm setter", setArmPID);

    // upperArmPos = daArmTab.add("Upper Arm Position", 0).getEntry();
    // lowerArmPos = daArmTab.add("Lower Arm Position", 0).getEntry();
    // wristPos = daArmTab.add("Wrist Position", 0).getEntry();
    // InstantCommand setArmPos = new InstantCommand(()-> m_ArmSubsystem.setPosition(lowerArmPos.getDouble(0),
    //  upperArmPos.getDouble(0), wristPos.getDouble(0)));
    // setArmPos.setName("Set Arm Position");
    // daArmTab.add("Arm Position Setter", setArmPos);

    InstantCommand setLowerArm = new InstantCommand(()-> m_ArmSubsystem.setLowerPosition(lowerArmPos.getDouble(0)));
    setLowerArm.setName("set lower arm");
    daArmTab.add("set lower Arm position", setLowerArm);
    
    InstantCommand setUpperArm = new InstantCommand(()-> m_ArmSubsystem.setUpperPosition(upperArmPos.getDouble(0)));
    setUpperArm.setName("set upper arm");
    daArmTab.add("set upper Arm position", setUpperArm);

    InstantCommand setWrist = new InstantCommand(()-> m_ArmSubsystem.setWristPosition(wristPos.getDouble(0)));
    setWrist.setName("set wrist arm");
    daArmTab.add("set wrist position", setWrist);

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

    InstantCommand resetArms = new InstantCommand(()->m_ArmSubsystem.resetArm());
    resetArms.setName("reset arms");
    daArmTab.add("reset arms", resetArms);

    InstantCommand setBrake = new InstantCommand(()-> m_ArmSubsystem.setBrake());
    setBrake.setName("set brake");
    WrapperCommand setBrakeCommand = setBrake.ignoringDisable(true);
    daArmTab.add("set brake", setBrakeCommand);

    InstantCommand setCoast = new InstantCommand(()-> m_ArmSubsystem.setCoast());
    WrapperCommand setCoastCommand = setCoast.ignoringDisable(true);
    daArmTab.add("set coast", setCoastCommand);

    // ShuffleboardTab maxspeedTab = Shuffleboard.getTab("max speed tab");
    
    // GenericEntry wristMinSpeed = maxspeedTab.add("wrist minimum speed", -.2).getEntry();
    // GenericEntry wristMaxSpeed = maxspeedTab.add("wrist maximum speed", .2).getEntry();

    // InstantCommand setWristSpeed = new InstantCommand(()-> m_ArmSubsystem.setWristOutputRange(wristMinSpeed.getDouble(0), wristMaxSpeed.getDouble(0)));
    // setWristSpeed.setName("set wristSpeed");
    // maxspeedTab.add("set wrist speed", setWristSpeed);

    // GenericEntry upperArmMinSpeed = maxspeedTab.add("upper arm minimum speed", -.6).getEntry();
    // GenericEntry upperArmMaxSpeed = maxspeedTab.add("upper arm maximum speed", .3).getEntry();

    // InstantCommand setUpperArmSpeed = new InstantCommand(()-> m_ArmSubsystem.setUpperArmOutputRange(upperArmMinSpeed.getDouble(0), upperArmMaxSpeed.getDouble(0)));
    // setUpperArmSpeed.setName("set upper arm speed");
    // maxspeedTab.add("set upper arm speed", setUpperArmSpeed);

    // GenericEntry lowerArmMinSpeed = maxspeedTab.add("lower arm minimum speed", -.4).getEntry();
    // GenericEntry lowerArmMaxSpeed = maxspeedTab.add("lower arm maximum speed", .8).getEntry();

    // InstantCommand setLowerArmSpeed = new InstantCommand(()-> m_ArmSubsystem.setLowerArmOutputRange(lowerArmMinSpeed.getDouble(0), lowerArmMaxSpeed.getDouble(0)));
    // setLowerArmSpeed.setName("set lower arm speed");
    // maxspeedTab.add("set lower arm speed", setLowerArmSpeed);

    GenericEntry rotateAngle = teleopTab.add("Go to Angle", 0).getEntry();
    teleopTab.add("gyro turn", new RotateToAngle(m_robotDrive, rotateAngle));

    teleopTab.add("gyro turn testing", new RotateToAngleTest(m_robotDrive, rotateAngle));

    teleopTab.add("pid gyro turn", new RotateToAnglePID(m_robotDrive, rotateAngle));

    // teleopTab.add("score backwards cube High", new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeHighUpperArm, 0, DriveConstants.m_backwardsScoreCubeWrist));
    // teleopTab.add("score backwards cube Medium", new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeMediumUpperArm, 0, DriveConstants.m_backwardsScoreCubMediumWrist));

    // teleopTab.add("flip over cone", flipConeUp);

    teleopTab.add("recreateSensor", new InstantCommand(()-> m_sensorSubsystem.reCreateSensor()));

    teleopTab.add("speed test", new SpeedTest(m_robotDrive));

    teleopTab.add("move a meter Forward", new MoveASmallDistancePid(m_robotDrive, 1, 0, 0));

    teleopTab.add("lower arm fast", new ArmLower(true, m_ArmSubsystem, 0, 0, 10));

    ShuffleboardTab movingTab = Shuffleboard.getTab("Moving Tab");
    GenericEntry xDistance = movingTab.add("x distance", 0).getEntry();
    GenericEntry yDistance = movingTab.add("y distance", 0).getEntry();
    GenericEntry heading = movingTab.add("heading", 0).getEntry();

    teleopTab.add("Go to position", new GoToPosition(m_robotDrive, xDistance, yDistance, heading));

    teleopTab.add("balance stuff test", new TestingAutoBalance(m_robotDrive));
    teleopTab.add("balance stuff test robot other way", new TestingAutoBalance(m_robotDrive, true));

    teleopTab.add("get cube", new MoveUntilCube(m_robotDrive, m_sensorSubsystem));

    diagnosticTab.add("Run Motor Diagnostics", new MotorDiagnostic(m_robotDrive, m_ArmSubsystem, intakeSubsystem));

    teleopTab.add("middle auto driving ", new MiddleAutonomousDriving(m_robotDrive, false));

    teleopTab.add("stuff", new custom_wheel_angleInput(m_robotDrive, 60, 60, 60, 60));

    teleopTab.add("driving a small distance ", new MoveATinyDistancePid(m_robotDrive, 0.1, 0, 180));

    teleopTab.add("drive till cone", new MoveUntilCone(m_robotDrive, m_sensorSubsystem));

    //teleopTab.add("test auto aim", testAutoMoveAim);
}

  // The driver's controller
  Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  // The copilot's controller

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
                driverJoystick.getPOV(), true);},
            m_robotDrive));
        
    ledSubsystem.setDefaultCommand(new RunCommand(()->
      ledSubsystem.setTheMode(intakeSubsystem.getScoreMode()), ledSubsystem));
    intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(intakeSubsystem));
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
    new JoystickButton(driverJoystick, 13)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(driverJoystick, 1)
        .whileTrue(fieldDriveOnOrOff);

    new JoystickButton(driverJoystick, 5).onTrue(testAutoScoreTop);
    // new JoystickButton(driverJoystick, 10).onTrue(testAutoScoreMedium);
    new JoystickButton(driverJoystick, 10).onTrue(new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.mS_ArmSetPointUpper, DriveConstants.mS_ArmSetPointLower, DriveConstants.mS_ArmSetPointWrist, true).andThen(new ArmRaise(m_ArmSubsystem, DriveConstants.mS_ArmSetPointUpper, DriveConstants.mS_ArmSetPointLower, DriveConstants.mS_ArmSetPointWrist, true)));
    new JoystickButton(driverJoystick, 6).onTrue(new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeHighUpperArm, 0, DriveConstants.m_backwardsScoreCubeWrist));
    new JoystickButton(driverJoystick, 9).onTrue(new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeMediumUpperArm, 0, DriveConstants.m_backwardsScoreCubMediumWrist));

    StartEndCommand intakeOut = new StartEndCommand(() -> intakeSubsystem.setCone(), () -> intakeSubsystem.setStop(), intakeSubsystem);
    new JoystickButton(driverJoystick, 3).whileTrue(intakeOut);

    StartEndCommand intakeIn = new StartEndCommand(() -> intakeSubsystem.setCube(), () -> intakeSubsystem.setStop(), intakeSubsystem);
    new JoystickButton(driverJoystick, 4).whileTrue(intakeIn);
    
    new JoystickButton(driverJoystick, 15).onTrue(new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist).andThen(new ArmRaise(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist)));
    new JoystickButton(driverJoystick, 16).onTrue(new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.mS_ArmSetPointUpper, DriveConstants.mS_ArmSetPointLower, DriveConstants.mS_ArmSetPointWrist, true).andThen(new ArmRaise(m_ArmSubsystem, DriveConstants.mS_ArmSetPointUpper, DriveConstants.mS_ArmSetPointLower, DriveConstants.mS_ArmSetPointWrist, true)));

    new JoystickButton(driverJoystick, 2).onTrue(new ArmLower(m_ArmSubsystem, 0, 0, 10));

    new JoystickButton(driverJoystick, 8).onTrue(flipIntakeOut);

    new JoystickButton(driverJoystick, 12).onTrue(new FlipIntake(m_ArmSubsystem, DriveConstants.m_wristOverCone));

    new JoystickButton(driverJoystick, 11).onTrue(new ArmRaiseSubstation(m_ArmSubsystem, DriveConstants.m_upperArmFoldedBackwards, 0, DriveConstants.m_wristFoldedBackwards));
    new JoystickButton(driverJoystick, 14).whileTrue(new MoveArmToFeeder(m_ArmSubsystem, -81, 0, 35));

    new JoystickButton(driverJoystick, 7).onTrue(flipConeUp);//was on button 12

    // Xbox controller bindings

    new JoystickButton(copilotController, 1).whileTrue(intakeIn);
    new JoystickButton(copilotController, 2).whileTrue(intakeOut);

    new JoystickButton(copilotController, 6).whileTrue(new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristOutOverFingers));

    InstantCommand resetPoseToBeginning = new InstantCommand(
        ()-> m_robotDrive.resetOdometry(new Pose2d(0,0,new Rotation2d(Math.toRadians(180)))));
        
    new JoystickButton(copilotController, 3).onTrue(resetPoseToBeginning);
    new JoystickButton(copilotController, 4).onTrue(new MoveArmToFeeder(m_ArmSubsystem, -77, 0, 10));
  }
  
  FlipIntake flipIntakeOut = new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristOut);
  FlipIntake flipIntakeIn = new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristIn);
  InstantCommand stopRollers = new InstantCommand(()-> intakeSubsystem.setStop());
  InstantCommand suckInCone = new InstantCommand(()->intakeSubsystem.setCone());
  InstantCommand suckInCube = new InstantCommand(()->intakeSubsystem.setCube());

  SequentialCommandGroup scoreHighConeNoAim = new SequentialCommandGroup(
    new InstantCommand(()->intakeSubsystem.setCone(0.8)),
    new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new ArmRaiseForAuto(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new intakeInOrOut(intakeSubsystem, true, true),
    new ArmLower(m_ArmSubsystem, 0, 0, 10));

  SequentialCommandGroup scoreHighConeNoAimSomeRetract = new SequentialCommandGroup(
    new InstantCommand(()->intakeSubsystem.setCone(0.8)),
    new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new ArmRaiseForAuto(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new intakeInOrOut(intakeSubsystem, true, true),
    new ArmLower(m_ArmSubsystem, -90, 0, 10, true));

  SequentialCommandGroup scoreHighConeNoAimNoRetract = new SequentialCommandGroup(
    new InstantCommand(()->intakeSubsystem.setCone(0.8)),
    new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new ArmRaiseForAuto(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new intakeInOrOut(intakeSubsystem, true, true));

  SequentialCommandGroup scoreHighConeNoAimForBalancing = new SequentialCommandGroup(
    new InstantCommand(()->intakeSubsystem.setCone(0.8)),
    new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new ArmRaise(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new intakeInOrOut(intakeSubsystem, true, true),
    new ArmLower(m_ArmSubsystem, 0, 0, 10));

  SequentialCommandGroup scoreHighConeNoAimForBalancingRetractHalf = new SequentialCommandGroup(
    new InstantCommand(()->intakeSubsystem.setCone(0.8)),
    new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new ArmRaise(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new intakeInOrOut(intakeSubsystem, true, true),
    new ArmLower(m_ArmSubsystem, -45, 0, 10));

  SequentialCommandGroup scoreHighCubeNoAim = new SequentialCommandGroup(
    new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new ArmRaise(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new intakeInOrOut(intakeSubsystem, false, true),
    new ArmLower(m_ArmSubsystem, 0, 0, 10));

  SequentialCommandGroup scoreHighCubeNoAimForBalancing = new SequentialCommandGroup(
    new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new ArmRaise(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new intakeInOrOut(intakeSubsystem, false, true),
    new ArmLower(m_ArmSubsystem, 0, 0, 10));

    SequentialCommandGroup balanceAutonomous = new SequentialCommandGroup(
      scoreHighCubeNoAimForBalancing, 
      new MiddleAutonomousDriving(m_robotDrive),
      new WaitCommand(0.5),
      new DriveForTime(m_robotDrive, 0, 0.2, 2.3),
      new TestingAutoBalance(m_robotDrive),
      new SetDrivetrainXForTime(m_robotDrive));

  SequentialCommandGroup balanceAutonomousAndPickupCone = new SequentialCommandGroup(
    scoreHighConeNoAimForBalancingRetractHalf,// new RotateToAngle(m_robotDrive, 180),
    new MiddleAutonomousDriving(m_robotDrive),
    new ParallelCommandGroup(
      new RotateToAngle(m_robotDrive, 180),
      new ArmRaiseSubstation(m_ArmSubsystem, DriveConstants.m_upperArmFoldedBackwards, 0, DriveConstants.m_wristFoldedBackwards)),
    new InstantCommand(()->intakeSubsystem.setCone()),
    new WaitCommand(1),
    new MoveASmallDistance(m_robotDrive, 0.8, 180, 0.2),
    new RotateToAngle(m_robotDrive, 180),
    new ParallelCommandGroup(
      new MoveASmallDistance(m_robotDrive, 1.9, 0, 0.3),
      new ArmLower(m_ArmSubsystem, 0, 0, 10)),
    new AutoBalanceHelper(m_robotDrive),
    new SetDrivetrainXForTime(m_robotDrive)
    );

    SequentialCommandGroup newBalanceAutoAndPickupCone = new SequentialCommandGroup(
      new InstantCommand(()->m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      new ParallelCommandGroup(
        new custom_wheel_angleInput(m_robotDrive, 60, 60, 60, 60),
        new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeHighUpperArm, 0, DriveConstants.m_backwardsScoreCubeWrist)
      ),
      new intakeInOrOut(intakeSubsystem, false, true),
      new ParallelCommandGroup(
        new ArmLower(true, m_ArmSubsystem, 0, 0, 10),
        new MoveASmallDistancePid(m_robotDrive, 0.3, 0.5, 0)
      ),
      new MiddleAutonomousDriving(m_robotDrive, false),
      new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristOut),
      new InstantCommand(()->intakeSubsystem.setCone()),
      // new ParallelCommandGroup(
      //   new MoveASmallDistancePid(m_robotDrive, 1.2, 0, 0)
      // ),
      new MoveUntilCone(m_robotDrive, m_sensorSubsystem),
      new ParallelCommandGroup(
        new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristIn),
        new MoveASmallDistancePid(m_robotDrive, -2.8, 0, 0)
      ),
      new TestingAutoBalance(m_robotDrive, true)
    );

    SequentialCommandGroup newBalanceAutoAndPickupConeToRight = new SequentialCommandGroup(
      new InstantCommand(()->m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      new ParallelCommandGroup(
        new custom_wheel_angleInput(m_robotDrive, 110, 110, 110, 110),
        new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeHighUpperArm, 0, DriveConstants.m_backwardsScoreCubeWrist)
      ),
      new intakeInOrOut(intakeSubsystem, false, true),
      new ParallelCommandGroup(
        new ArmLower(true, m_ArmSubsystem, 0, 0, 10),
        new MoveASmallDistancePid(m_robotDrive, 0.3, -0.65, 0)
      ),
      new MiddleAutonomousDriving(m_robotDrive, false),
      new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristOut),
      new InstantCommand(()->intakeSubsystem.setCone()),
      // new ParallelCommandGroup(
      //   new MoveASmallDistancePid(m_robotDrive, 1.2, 0, 0)
      // ),
      new MoveUntilCone(m_robotDrive, m_sensorSubsystem),
      new ParallelCommandGroup(
        new FlipIntake(m_ArmSubsystem, 125),
        new MoveASmallDistancePid(m_robotDrive, -2.8, 0, 0)
      ),
      new ParallelCommandGroup(
        new TestingAutoBalance(m_robotDrive, true),
        new InstantCommand(()->intakeSubsystem.setCone(1))
      )
    );

    SequentialCommandGroup newBalanceAutoAndPickupCubeToRight = new SequentialCommandGroup(
      new InstantCommand(()->m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      new ParallelCommandGroup(
        new custom_wheel_angleInput(m_robotDrive, 110, 110, 110, 110),
        new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeHighUpperArm, 0, DriveConstants.m_backwardsScoreCubeWrist)
      ),
      new intakeInOrOut(intakeSubsystem, true, true),
      new ParallelCommandGroup(
        new ArmLower(true, m_ArmSubsystem, 0, 0, 10),
        new MoveASmallDistancePid(m_robotDrive, 0.3, -0.65, 0)
      ),
      new MiddleAutonomousDriving(m_robotDrive, false),
      new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristOut),
      new InstantCommand(()->intakeSubsystem.setCone()),
      // new ParallelCommandGroup(
      //   new MoveASmallDistancePid(m_robotDrive, 1.2, 0, 0)
      // ),
      new MoveUntilCubeSetSpeed(m_robotDrive, m_sensorSubsystem, 1.143),
      new ParallelCommandGroup(
        new FlipIntake(m_ArmSubsystem, 125),
        new RotateToAnglePID(m_robotDrive, 180)
      ),
      new MoveASmallDistancePid(m_robotDrive, -2.8, 0, 180),
      new ParallelCommandGroup(
        new TestingAutoBalance(m_robotDrive, false),
        new InstantCommand(()->intakeSubsystem.setCone(1))
      )
    );
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selected = m_chooser.getSelected();
    System.out.println(selected.getName());

    return selected;
  }


  //The commands for the auto aim
  SequentialCommandGroup testAutoScoreTop = new SequentialCommandGroup(
    new InstantCommand(()->{
      if(intakeSubsystem.getScoreMode().equals("cone")){
        intakeSubsystem.setMotor(-1);
      }}),
    new ParallelCommandGroup(
      new SequentialCommandGroup(
        new MoveASmallDistance(m_robotDrive, 0.0762, 180, 0.2),
        new RotateToAngle(m_robotDrive, 180),
        new readLimelight(limelight_test, intakeSubsystem),
        new WaitCommand(.1),
        new ExAutoAim(limelight_test, m_robotDrive, m_sensorSubsystem, intakeSubsystem),
        //new MoveASmallDistancePid(m_robotDrive, 0.076, 0, 180)
        // new MoveATinyDistancePid(m_robotDrive, -0.1, 0, 180)
        new custom_wheel_angleInputFast(m_robotDrive, 0, 0, 0, 0),
        new MoveASmallDistance(m_robotDrive, 0.09, 0, 0.15),
        new RotateToAnglePIDAgressive(m_robotDrive, 180)
        ),
      new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist)),
    new ArmRaise(m_ArmSubsystem, DriveConstants.hS_ArmSetPointUpper, DriveConstants.hS_ArmSetPointLower, DriveConstants.hS_ArmSetPointWrist),
    new AutoIntakeInOrOut(intakeSubsystem, true),
    new InstantCommand(()->intakeSubsystem.setScoreModeNone()),
    new ArmLower(m_ArmSubsystem, 0, 0, 10)
  );

  SequentialCommandGroup testAutoMoveAim = new SequentialCommandGroup(
    new InstantCommand(()->{
      if(intakeSubsystem.getScoreMode().equals("cone")){
        intakeSubsystem.setMotor(-1);
      }}),
    new ParallelCommandGroup(
    new SequentialCommandGroup(
      new MoveASmallDistance(m_robotDrive, 0.0762, 180, 0.2),
      new RotateToAngle(m_robotDrive, 180),
      new readLimelight(limelight_test, intakeSubsystem),
      new WaitCommand(.1),
      new ExAutoAim(limelight_test, m_robotDrive, m_sensorSubsystem, intakeSubsystem),
      new custom_wheel_angleInputFast(m_robotDrive, 0, 0, 0, 0),
      new MoveASmallDistance(m_robotDrive, 0.09, 0, 0.2)
        )));

  SequentialCommandGroup testAutoScoreMedium = new SequentialCommandGroup(
    new InstantCommand(()->{
      if(intakeSubsystem.getScoreMode().equals("cone")){
        intakeSubsystem.setMotor(-0.8);
      }}),
    new ParallelCommandGroup(
      new SequentialCommandGroup(
        new MoveASmallDistance(m_robotDrive, 0.0762, 180, 0.2),
        new readLimelight(limelight_test, intakeSubsystem),
        new RotateToAngle(m_robotDrive, 180),
        new WaitCommand(.1),
        new ExAutoAim(limelight_test, m_robotDrive, m_sensorSubsystem, intakeSubsystem),
        new custom_wheel_angleInputFast(m_robotDrive, 0, 0, 0, 0),
        new MoveASmallDistance(m_robotDrive, 0.09, 0, 0.15)),
      new ArmRaisePrepare(m_ArmSubsystem, DriveConstants.mS_ArmSetPointUpper, DriveConstants.mS_ArmSetPointLower, DriveConstants.mS_ArmSetPointWrist, true)),
    new ArmRaise(m_ArmSubsystem, DriveConstants.mS_ArmSetPointUpper, DriveConstants.mS_ArmSetPointLower, DriveConstants.mS_ArmSetPointWrist, true),
    new AutoIntakeInOrOut(intakeSubsystem, true),
    new InstantCommand(()->intakeSubsystem.setScoreModeNone()),
    new ArmLower(m_ArmSubsystem, 0, 0, 10)
  );

  SequentialCommandGroup flipConeUp = new SequentialCommandGroup(
    new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristOut - 40),
    new DriveForTime(m_robotDrive, 180, 0.15, 0.75),
    new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristOut),
    new InstantCommand(()->intakeSubsystem.setCone()),
    new ParallelRaceGroup(
    new DriveForTime(m_robotDrive, 0, 0.15, 1.25),
    new RunIntakeUntilStall(m_ArmSubsystem, intakeSubsystem, true))
  );

  private final String middleAuto = "middle auto balance";
  private final String middleAutoAndPickup = "middle auto balance and Pickup";
  private final String blueLeftAuto3Piece = "Left 3 piece";
  private final String blueRIghtAuto3Piece = "Right 3 Piece";
  private final String bumpLeftAuto3Piece = "Left 3 piece Bump";
  private final String bumpRightAuto3Piece = "Right 3 Piece Bump";
  private final String left2PieceBalance = "Left 2 Piece Balance";
  private final String blueRIghtAuto2Piece = "Blue Right Auto 2 Piece";
  private final String left3PiecePlus = "Left 3 piece plus";
  private final String Right3PiecePlus = "Right 3 Piece plus";
  private final String rightEscape = "backup exit right";
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public void autoChooserInit() {
    m_chooser.setDefaultOption("Score High Cone", new InstantCommand());
    m_chooser.addOption("score High cone", scoreHighConeNoAim);
    m_chooser.addOption("Score high Cube", scoreHighCubeNoAim);
    m_chooser.addOption("score-pickup left-balance", newBalanceAutoAndPickupCone);
    m_chooser.addOption("score-pickup right-balance", newBalanceAutoAndPickupConeToRight);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("ScoreNoAiming", scoreHighConeNoAim);
    eventMap.put("ScoreNoAimingSomeRetract", scoreHighConeNoAimSomeRetract);
    eventMap.put("ScoreNoAimingNoRetract", scoreHighConeNoAimNoRetract);
    eventMap.put("ScoreAiming", testAutoScoreTop);
    eventMap.put("FlipIntakeOut", flipIntakeOut);
    eventMap.put("TurnOnRollers", suckInCone);
    eventMap.put("FlipIntakeIn", flipIntakeIn);
    eventMap.put("StopRollers", stopRollers);
    eventMap.put("extendArmBackwards", new ArmRaiseSubstation(m_ArmSubsystem, DriveConstants.m_upperArmFoldedBackwards, 0, DriveConstants.m_wristFoldedBackwards));
    eventMap.put("RetractArm", new ArmLower(m_ArmSubsystem, 0, 0, 10));
    eventMap.put("RetractArmFast", new ArmLower(true, m_ArmSubsystem, 0, 0, 10));
    eventMap.put("DriveIntoWall", new DriveForTime(m_robotDrive, 0, 0.25, 0.55));
    eventMap.put("DriveIntoWallBackwards", new DriveForTime(m_robotDrive, 180, 0.25, 0.55));
    eventMap.put("shootOutCone", new InstantCommand(()->intakeSubsystem.setCube(0.6)));
    eventMap.put("extendCubeHigh",new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeHighUpperArm, 0, DriveConstants.m_backwardsScoreCubeWrist));
    eventMap.put("extendCubeMed",new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeMediumUpperArm, 0, DriveConstants.m_backwardsScoreCubMediumWrist));
    eventMap.put("shootCube", new intakeInOrOut(intakeSubsystem, false, true));
    eventMap.put("suckInCube", suckInCube);
    eventMap.put("balanceChargeStationIntakeForward", new TestingAutoBalance(m_robotDrive, true));
    eventMap.put("moveForwardUntilCube", new MoveUntilCube(m_robotDrive, m_sensorSubsystem));
    eventMap.put("stopUnlessCone", new StopUntilCone(m_robotDrive, m_sensorSubsystem));
    eventMap.put("shootCubeSlow", new ShootCubeSlow(intakeSubsystem));
    eventMap.put("Extend and Score", new ParallelCommandGroup(
      new custom_wheel_angleInput(m_robotDrive, 0, 0, 0, 0),
      new SequentialCommandGroup(
        new ArmRaiseScoringCube(m_ArmSubsystem, DriveConstants.m_backwardsScoreCubeHighUpperArm, 0, DriveConstants.m_backwardsScoreCubeWrist),
        new intakeInOrOut(intakeSubsystem, false, true))));
    eventMap.put("wristOverFingers", new FlipIntake(m_ArmSubsystem, DriveConstants.m_WristOutOverFingers));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      m_robotDrive::getPose, // Pose2d supplier
      m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(blueRIghtAuto3Piece, new PathConstraints(4, 1.9));
    m_chooser.addOption("Red Right 2.5 Piece", autoBuilder.fullAuto(pathGroup));
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup(bumpLeftAuto3Piece, new PathConstraints(4, 1.7));
    m_chooser.addOption("Red bump left side 2.5 Piece", autoBuilder.fullAuto(pathGroup1));
    List<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup(bumpRightAuto3Piece, new PathConstraints(4, 1.7));
    m_chooser.addOption("Blue bump right side 2.5 piece", autoBuilder.fullAuto(pathGroup2));
    List<PathPlannerTrajectory> pathGroup3 = PathPlanner.loadPathGroup(blueLeftAuto3Piece, new PathConstraints(4, 1.9));
    m_chooser.addOption("Blue Left Auto 2.5 piece", autoBuilder.fullAuto(pathGroup3));
    List<PathPlannerTrajectory> pathGroup4 = PathPlanner.loadPathGroup(left3PiecePlus, new PathConstraints(4, 2.3));
    m_chooser.addOption("Blue Left Auto 3 piece", autoBuilder.fullAuto(pathGroup4));
    List<PathPlannerTrajectory> pathGroup5 = PathPlanner.loadPathGroup(rightEscape, new PathConstraints(4, 2));
    m_chooser.addOption("escape", autoBuilder.fullAuto(pathGroup5));
    List<PathPlannerTrajectory> pathGroup6 = PathPlanner.loadPathGroup(Right3PiecePlus, new PathConstraints(4, 2.3));
    m_chooser.addOption("Red Right Auto 3 piece", autoBuilder.fullAuto(pathGroup6));
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  public void freezeArm(){
    m_ArmSubsystem.stopMotors();
  }

  public void resetArm(){
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))));
    // m_ArmSubsystem.resetArm();
  }

  public void setX(){
    m_robotDrive.setX();
  }

  public boolean getIfBallancing(){
    return false;
  }

  public void checkConnection(){
    boolean armFailure = m_ArmSubsystem.checkConnection();
    boolean driveFailure = m_robotDrive.checkConnection();
    boolean intakeFailure = intakeSubsystem.checkConnection();
    
    if(armFailure || driveFailure || intakeFailure){
      ledSubsystem.setError(true);
    }
  }
}
 