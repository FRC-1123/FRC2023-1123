// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MAXSwerveModule;

import com.ctre.phoenix.CANifier.GeneralPin;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GoToPosition extends CommandBase {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  PIDController m_RotationController;
  PIDController m_xPidController;
  PIDController m_yPidController;
  GenericEntry xDistance;
  GenericEntry yDistance;
  GenericEntry heading;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToPosition(DriveSubsystem subsystem, double xDistance, double yDistance, double heading) {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    setUpController(xDistance, yDistance, heading);
    addRequirements(subsystem);
  }

  public GoToPosition(DriveSubsystem subsystem, GenericEntry xDistance, GenericEntry yDistance, GenericEntry heading) {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    setUpController(xDistance.getDouble(0), yDistance.getDouble(0), heading.getDouble(0));
    addRequirements(subsystem);
  }

  private void setUpController(double xDistance, double yDistance, double heading){
    m_RotationController = new PIDController(0.01, 0, 0);
    m_RotationController.enableContinuousInput(-180, 180);
    m_RotationController.setSetpoint(heading);
    m_RotationController.setTolerance(1);
    m_xPidController = new PIDController(1, 0, 0);
    m_xPidController.setTolerance(0.0508);
    m_yPidController = new PIDController(1, 0, 0);
    m_yPidController.setTolerance(0.0508);
    m_xPidController.setSetpoint(xDistance);
    m_yPidController.setSetpoint(yDistance);
  }
  int timesDone;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(xDistance != null){
      m_xPidController.setSetpoint(xDistance.getDouble(0));
      m_yPidController.setSetpoint(yDistance.getDouble(0));
      m_RotationController.setSetpoint(heading.getDouble(0));
    }
    timesDone = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(MathUtil.clamp(m_xPidController.calculate(m_subsystem.getPose().getX()),-0.5,0.5),
     MathUtil.clamp(m_yPidController.calculate(m_subsystem.getPose().getY()), -0.5, 0.5),
      MathUtil.clamp(m_RotationController.calculate(m_subsystem.getPose().getRotation().getDegrees()), -0.5, 0.5), true);

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
    boolean atSetpoint = m_RotationController.atSetpoint();
    if(atSetpoint && timesDone > 10){
      return true;
    }
    if(atSetpoint){
      timesDone++;
      return false;
    }
    timesDone = 0;
    return false;
  }
}