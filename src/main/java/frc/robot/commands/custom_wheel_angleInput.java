// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class custom_wheel_angleInput extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  int m_fRightAngle;
  int m_rRightAngle;
  int m_fLeftAngle;
  int m_rLeftAngle;
  double startTime;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public custom_wheel_angleInput(DriveSubsystem subsystem, int fRightAngle, int rRightAngle, int fLeftAngle, 
  int rLeftAngle) {
    m_subsystem = subsystem;
    m_fLeftAngle = fLeftAngle;
    m_fRightAngle = fRightAngle;
    m_rLeftAngle = rLeftAngle;
    m_rRightAngle = rRightAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setWheelAngle(m_fRightAngle, m_rRightAngle,
     m_fLeftAngle, m_rLeftAngle);
    
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp()-startTime > 1){
      return true;
    }
    return false;
  }
}
