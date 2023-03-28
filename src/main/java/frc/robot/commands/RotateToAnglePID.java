// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateToAnglePID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  int time = 0;
  double angle;
  GenericEntry angleEntry = null;
  int timesDone = 0;
  PIDController m_RotationController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateToAnglePID(DriveSubsystem subsystem, double angle) {
    m_subsystem = subsystem;
    this.angle = angle;
    m_RotationController = new PIDController(0.05, 0, 0);
    m_RotationController.enableContinuousInput(-180, 180);
    m_RotationController.setTolerance(1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public RotateToAnglePID(DriveSubsystem subsystem, GenericEntry angle) {
    m_subsystem = subsystem;
    this.angleEntry = angle;
    m_RotationController = new PIDController(0.05, 0, 0);
    m_RotationController.enableContinuousInput(-180, 180);
    m_RotationController.setTolerance(1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    time = 0;
    timesDone = 0;
    if(angleEntry != null){
      angle = angleEntry.getDouble(0);
    }
    if(angle > 180){
      angle -=360;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time++;
    // move();
    m_subsystem.drive(0, 0, m_RotationController.calculate(m_subsystem.getPose().getRotation().getDegrees(), angle), false);
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
    // logger.info("delta " + delta);
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
