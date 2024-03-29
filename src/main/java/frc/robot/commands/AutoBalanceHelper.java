// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoBalanceHelper extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  int time = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoBalanceHelper(DriveSubsystem subsystem) {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = m_subsystem.getPitch();
    if(Math.abs(pitch)> 9){
      if(pitch>0){
        if(Math.abs(pitch) > 10){
          m_subsystem.drive(.09, 0, 0, false);
        }
        else{
          m_subsystem.drive(.03, 0, 0, false);
        }
      }
      else{
        if(Math.abs(pitch) > 10){
          m_subsystem.drive(-.09, 0, 0, false);
        }
        else{
          m_subsystem.drive(-.03, 0, 0, false);
        }
        // m_subsystem.drive(-.1, 0, 0, false);
      }

    }
    else{
      m_subsystem.setX();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
