// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateToAngleTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  int time = 0;
  double angle;
  GenericEntry angleEntry = null;
  int timesDone = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateToAngleTest(DriveSubsystem subsystem, double angle) {
    m_subsystem = subsystem;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public RotateToAngleTest(DriveSubsystem subsystem, GenericEntry angle) {
    m_subsystem = subsystem;
    this.angleEntry = angle;
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
    move();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double delta = 0;
    double angle = m_subsystem.getPose().getRotation().getDegrees();
    delta = angle-this.angle;
    // logger.info("delta " + delta);
    if(Math.abs(delta) < 1.5 && timesDone > 10){
      return true;
    }
    if(Math.abs(delta) < 1.5){
      timesDone++;
      return false;
    }
    timesDone = 0;
    return false;
  }

  private void move(){
    double gyroAngle = m_subsystem.getPose().getRotation().getDegrees();
    double delta = gyroAngle - angle;
    if(delta < 0){
      delta = Math.abs(delta);
      if(delta < 180){
        driving(delta, 1);
      }
      else{
        delta = 360 - delta;
        driving(delta, -1);
      }
    }
    else{
      if(delta < 180){
        driving(delta, -1);
      }
      else{
        delta = 360 - delta;
        driving(delta, 1);
      }
    }
  }

  private void driving(double angle, int direction){ 
    angle = Math.abs(angle);
    if(angle > 50)
          m_subsystem.drive(0, 0, direction*1, false);//fast
        else if(angle < 10)
        m_subsystem.drive(0, 0, direction*.04, false);//slow
        else 
        m_subsystem.drive(0, 0, direction*.18, false);//medium
  }
}
