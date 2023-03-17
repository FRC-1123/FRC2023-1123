// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveForTime extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  int time = 0;
  int direction;
  double speed;
  int driveTime;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForTime(DriveSubsystem subsystem, int direction, double speed, double driveTime) {
    m_subsystem = subsystem;
    this.direction = direction;
    this.speed = speed;
    this.driveTime = (int)(driveTime*50);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0;
    switch(direction){
      case 0: m_subsystem.drive(speed, 0, 0, false);
        break;
      case 90: m_subsystem.drive(0, speed, 0, false);
        break;
      case 180: m_subsystem.drive(-speed, 0, 0, false);
        break;
      case 270: m_subsystem.drive(0, -speed, 0, false);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time > driveTime){
      return true;
    }
    return false;
  }
}
