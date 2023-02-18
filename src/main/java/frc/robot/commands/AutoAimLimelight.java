package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAimLimelight extends CommandBase {
    private DriveSubsystem drive;
    private LimelightSubsystem limelight;
    private Command autoAim;
    public AutoAimLimelight(DriveSubsystem drive, LimelightSubsystem limelight){
        this.drive = drive;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        Pose2d pose = drive.getPose();
        autoAim = generateSwerveCommand(pose, new Pose2d(pose.getX(), pose.getY()+limelight.getTangent(), pose.getRotation()));
        autoAim.schedule();

        // some testing
        System.out.println("switching pipeline to 2");
        limelight.setPipeline(2);
        System.out.println("X = "+limelight.lime_x);
        System.out.println("Y = "+limelight.lime_y);

        System.out.println("switching pipeline to 1");
        limelight.setPipeline(1);
        System.out.println("X = "+limelight.lime_x);
        System.out.println("Y = "+limelight.lime_y);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO uncomment to make sure things stop
    //drive.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !autoAim.isScheduled();
  }

  private Command generateSwerveCommand(Pose2d startPosition, Pose2d endPosition){
    System.out.println(startPosition.getRotation().getDegrees() + " stuff " + endPosition.getRotation().getDegrees());
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            startPosition,
            List.of(),
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
            drive::setModuleStates,
            drive);
        return swerveControllerCommand;
  }
    
}
