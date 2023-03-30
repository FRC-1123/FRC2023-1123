// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MAXSwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveASmallDistancePid extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  double xDistance;
  double yDistance;
  double speed;
  // double maxSpeed;
  // double heading;
  PIDController m_RotationController;
  PIDController m_xPidController;
  PIDController m_yPidController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveASmallDistancePid(DriveSubsystem subsystem, double xDistance, double yDistance, double heading) {
    m_subsystem = subsystem;
    this.xDistance = xDistance;
    this.yDistance = yDistance;
    // this.maxSpeed = maxSpeed;
    // this.heading = heading;
    m_RotationController = new PIDController(0.01, 0, 0);
    m_RotationController.enableContinuousInput(-180, 180);
    m_RotationController.setSetpoint(heading);
    m_RotationController.setTolerance(1);
    m_xPidController = new PIDController(1, 0, 0);
    m_xPidController.setTolerance(0.0508);
    m_yPidController = new PIDController(1, 0, 0);
    m_yPidController.setTolerance(0.0508);
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(subsystem);
  }
  double initialX = 0;
  double initialY = 0;
  int timesDone;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialX = m_subsystem.getPose().getX();
    initialY = m_subsystem.getPose().getY();
    m_xPidController.setSetpoint(xDistance + initialX);
    m_yPidController.setSetpoint(yDistance + initialY);
    timesDone = 0;
    speed = 0.45;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(MathUtil.clamp(m_xPidController.calculate(m_subsystem.getPose().getX()),-speed,speed),
     MathUtil.clamp(m_yPidController.calculate(m_subsystem.getPose().getY()), -speed, speed),
      MathUtil.clamp(m_RotationController.calculate(m_subsystem.getPose().getRotation().getDegrees()), -speed, speed), true);

    // m_subsystem.drive(m_xPidController.calculate(m_subsystem.getPose().getX()-initialX),
    // m_yPidController.calculate(m_subsystem.getPose().getY()-initialY),
    // 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSetpointX = m_xPidController.atSetpoint();
    boolean atSetpointY= m_yPidController.atSetpoint();
    boolean atSetpointR = m_RotationController.atSetpoint();
    if(atSetpointX && atSetpointY && atSetpointR && timesDone > 10){
      return true;
    }
    if(atSetpointX && atSetpointY && atSetpointR){
      timesDone++;
      return false;
    }
    timesDone = 0;
    return false;
  }
}