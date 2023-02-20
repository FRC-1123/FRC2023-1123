package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    int time = 0;

    @Override
    public void initialize() {

        Pose2d pose = drive.getPose();
        autoAim = generateSwerveCommand(pose, new Pose2d(pose.getX(), pose.getY()+limelight.getTangent(), pose.getRotation()));
        autoAim.schedule();

        // some testing
        changePipeline(1);
        System.out.println("--initial reading--");
        System.out.println("X = "+limelight.lime_x);
        System.out.println("Y = "+limelight.lime_y);
        //limelight.setPipeline(2);
        //System.out.println("X = "+limelight.lime_x);
        //System.out.println("Y = "+limelight.lime_y);

        //System.out.println("switching pipeline to 1");
        //limelight.setPipeline(1);
        //NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("pipeline").setNumber(1);
        //System.out.println("X = "+limelight.lime_x);
        //System.out.println("Y = "+limelight.lime_y);

        
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* THE GRAND MASTER PLAN FOR AUTO SCORING
    
     * STEP 1
     * See what position the arm is at and whether we're scoring a cone or cube
     * the arm position check tells us at what place we are scoring at BASE, MIDDLE, or TOP
     * move robot slightly backwards, giving room to manuavere
     * 
     * STEP 2 (if scoring bottom, skip this step)
     * determine if there is a target in sight. if not, break and give an error message
     */
    if(scoringOnBottom()==false){
        if(targetInSight()==true){
            double tangent = limelight.getTangent();
        }
        else{
            ouputError();
            // some how cancel the command
        }
    }
     /* get the tangent of the target relative to the robot
     * compute that into a move, taking into account where the object is in the intake, and execute
     * do a check with the limelight to see if the target is withen the acceptable margin of error
     * if above is false, go back to STEP 2 and try again
     * 
     * STEP 3
     * score the piece!
     */
        
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO uncomment to make sure things stop
    //drive.drive(0,0,0,false);
    changePipeline(1);
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

  private void changePipeline(int number){
    NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("pipeline").setNumber(number);
    for(int i = 0; i<10; i++){}
  }

  private void reportNoTarget(){
    System.out.println("No target detected!");
  }

  private void autoAimCone(){
    System.out.println("Place a cone!");
  }

  private void autoAimCube(){
    System.out.println("Drop a cube!");
  }

  private boolean scoringOnBottom(){
    return false;
  }

  private boolean targetInSight(){
    return false;
  }

  private void ouputError(){
    System.out.println("No target found!");
  }
    
}
