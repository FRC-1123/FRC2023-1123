// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SpeedTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  double time = 0;
  double maxSpeed = 0;
  double averageAcceleration = 0;
  int outputs=0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SpeedTest(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
    m_subsystem.drive(1, 0, 0, false);
    outputs = 0;
    lastRun = Timer.getFPGATimestamp();
    lastSpeed = 0;
  }
  double lastRun;
  double lastSpeed;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    outputs++;
    double averageSpeed = m_subsystem.getAverage();
    if(averageSpeed > maxSpeed){
      maxSpeed = averageSpeed;
    }
    averageAcceleration = averageSpeed/(Timer.getFPGATimestamp()-time);
    if(outputs > 5){
      System.out.println("average total acceleration " + averageAcceleration);
      double recentAcceleration = (averageSpeed-lastSpeed)/(Timer.getFPGATimestamp()-lastRun);
      System.out.println("average recent acceleration " + recentAcceleration);
      System.out.println("speed " + averageSpeed);
      outputs = 0;
      lastRun = Timer.getFPGATimestamp();
      lastSpeed = averageSpeed;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
    System.out.println("maxSpeed = " + maxSpeed + " average total acceleration " + averageAcceleration);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp()-time > 5){
      return true;
    }
    return false;
  }
}
