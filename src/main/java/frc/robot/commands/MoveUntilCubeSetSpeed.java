// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SensorSubsystem;

/** An example command that uses an example subsystem. */
public class MoveUntilCubeSetSpeed extends CommandBase {
  private final DriveSubsystem drive;
  private final SensorSubsystem sensor;
  private Pose2d initalPose;
  double distance_limit;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveUntilCubeSetSpeed(DriveSubsystem drive, SensorSubsystem sensor, double distance_limit){
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.sensor = sensor;
    this.distance_limit = distance_limit;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initalPose = drive.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!sensor.isCube()){
      drive.drive(0.3, 0, 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("cube " + sensor.isCube());
    if(sensor.isCube()){
      return true;
    }
    Transform2d currentPose = drive.getPose().minus(initalPose);
    System.out.println("distance " + (Math.abs(currentPose.getX()) + Math.abs(currentPose.getY())));
    if(Math.abs(currentPose.getX()) + Math.abs(currentPose.getY()) > distance_limit){
      return true;
    }
    return false;
  }
}
