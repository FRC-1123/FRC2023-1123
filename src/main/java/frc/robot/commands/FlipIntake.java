// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class FlipIntake extends CommandBase {
  private final ArmSubsystem m_armSubsystem;

  double m_wristPos;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlipIntake(ArmSubsystem armed, double wP){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
  m_armSubsystem = armed;
  m_wristPos = wP;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setWristPosition(m_wristPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_armSubsystem.getWristPosition()- m_wristPos)<5.0){
      return true;
    }
  return false;
  }
}
