// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveASmallDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  int time = 0;
  double distance;
  int direction;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveASmallDistance(DriveSubsystem subsystem, double distance, int direction) {
    m_subsystem = subsystem;
    this.distance = distance;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetOdometry(new Pose2d(0,0, m_subsystem.getPose().getRotation()));
    time = 0;
    switch(direction){
      case 0: m_subsystem.drive(.05, 0, 0, false);
        break;
      case 90: m_subsystem.drive(0, 0.05, 0, false);
        break;
      case 180: m_subsystem.drive(-.05, 0, 0, false);
        break;
      case 270: m_subsystem.drive(0, -.05, 0, false);
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
    // m_subsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_subsystem.getPose().getX()) + Math.abs(m_subsystem.getPose().getY()) > distance){
      return true;
    }
    return false;
  }
}