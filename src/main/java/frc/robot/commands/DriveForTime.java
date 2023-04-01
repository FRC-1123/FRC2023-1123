// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveForTime extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  double time = 0;
  int direction;
  double speed;
  double driveTime;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForTime(DriveSubsystem subsystem, int direction, double speed, double driveTime) {
    m_subsystem = subsystem;
    this.direction = direction;
    this.speed = speed;
    this.driveTime = driveTime;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
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
    if(Timer.getFPGATimestamp()-time > driveTime){
      System.out.println("in drive for time finished speed " + speed + ". time " + driveTime);
      return true;
    }
    return false;
  }
}
