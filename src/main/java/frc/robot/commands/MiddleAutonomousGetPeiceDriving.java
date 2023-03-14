// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class MiddleAutonomousGetPeiceDriving extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem drive;
  int time = 0;
  Command move;
  double wantedRotation = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MiddleAutonomousGetPeiceDriving(DriveSubsystem drive) {
    this.drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetOdometry(new Pose2d(0,0,drive.getPose().getRotation()));
    move = generateSwerveCommand(new Pose2d(0,0,drive.getPose().getRotation()), new Pose2d(0,0, new Rotation2d(Math.toRadians(180))), new Translation2d(1,0));
    move.schedule();
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
    if(!move.isScheduled()){
      return true;
    }
    return false;
  }

  private Command generateSwerveCommand(Pose2d startPosition, Pose2d endPosition, Translation2d waypoint){
    System.out.println(startPosition.getRotation().getDegrees() + " stuff " + endPosition.getRotation().getDegrees());
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            2,
            2)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            startPosition,
            List.of(waypoint),
            endPosition,
            config);
    
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            this::getWantedRotation,
            drive::setModuleStates,
            drive);
        return swerveControllerCommand;
  }

  public Rotation2d getWantedRotation(){
    return new Rotation2d(Math.toRadians(wantedRotation));
  }
}
