// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SensorSubsystem;

/** An example command that uses an example subsystem. */
public class StopUntilCone extends CommandBase {
  private final DriveSubsystem drive;
  private final SensorSubsystem sensor;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopUntilCone(DriveSubsystem drive, SensorSubsystem sensor){
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.sensor = sensor;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(sensor.isCone()){
      return true;
    }
    return false;
  }
}
