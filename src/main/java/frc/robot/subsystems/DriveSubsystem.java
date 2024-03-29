// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  public final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  public final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId, 
      DriveConstants.kFrontRightTurningCanId, 
      DriveConstants.kFrontRightChassisAngularOffset);

  public final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  public final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(Port.kUSB1);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
      PIDController m_RotationController;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_RotationController = new PIDController(0.01, 0, 0);
    m_RotationController.enableContinuousInput(-180, 180);
    m_RotationController.setTolerance(1);
  }


  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getGyroData()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
        
    time++;
    // SmartDashboard.putNumber("Headed Gyro", getGyroData());
    SmartDashboard.putNumber("pitch", getPitch());
    
    Pose2d currentPose = getPose();

    SmartDashboard.putNumber("pose X", currentPose.getX());
    SmartDashboard.putNumber("pose Y", currentPose.getY());
    SmartDashboard.putNumber("pose angle", currentPose.getRotation().getDegrees());
  }

  int time = 0;

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroData()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if(xSpeed == 0 && ySpeed == 0 && rot == 0){
      m_frontLeft.stopMotors();
      m_frontRight.stopMotors();
      m_rearLeft.stopMotors();
      m_rearRight.stopMotors();
      return;
    }
    
    // Adjust input based on max speed
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(double xSpeed, double ySpeed, double rot, int pov, boolean fieldRelative) {
    if(pov != -1){
      if(pov < 180){
        pov = -pov;
      }
      else if(pov > 180){
        pov = 360 - pov;
      }
      if(Math.abs(m_RotationController.getPositionError())<6){
        m_RotationController.setP(0.02);
      }
      else{
        m_RotationController.setP(0.01);
      }
      if(Math.abs(pov)<90){
        pov = 0;
      }
      else if(Math.abs(pov)> 90){
        pov = 180;
      }
      drive(xSpeed, ySpeed, m_RotationController.calculate(getPose().getRotation().getDegrees(), pov), fieldRelative);
    }
    else{
      //System.out.println("pov " + pov);
      drive(xSpeed, ySpeed, rot, fieldRelative);
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setWheelAngle(long f_RightAngle, long r_RightAngle, long f_LeftAngle, long r_LeftAngle) {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(f_LeftAngle)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(f_RightAngle)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(r_LeftAngle)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(r_RightAngle)));
  }


  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
    System.out.println("in reset encoders");
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    //m_gyro.calibrate();
    System.out.println("in zero heading");
    resetTime = time;
  }
  int resetTime = 0;
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getGyroData()).getDegrees();
  }
  double lastHeading = 0;

  public double getGyroData(){
    if(time - resetTime < 25){
      return lastHeading;
    }
    else{
      double angle = -m_gyro.getAngle();
      lastHeading = angle;  
      return angle;
    }
  }

  public double getAverage(){
    SwerveModuleState a = m_frontLeft.getState();
    SwerveModuleState b = m_frontRight.getState();
    SwerveModuleState c = m_rearLeft.getState();
    SwerveModuleState d = m_rearRight.getState();

    double total = Math.abs(a.speedMetersPerSecond) + Math.abs(b.speedMetersPerSecond) + Math.abs(c.speedMetersPerSecond) + Math.abs(d.speedMetersPerSecond);
    total = total / 4;
    return total;
  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

public double getPitch() {
  // return m_gyro.getPitch();
  return -m_gyro.getRoll();
}

public boolean checkConnection(){
  boolean m_frontLeftFail = m_frontLeft.checkConnection();
  boolean m_frontRightFail = m_frontRight.checkConnection();
  boolean m_rearRightFail = m_rearRight.checkConnection();
  boolean m_rearLeftFail = m_rearLeft.checkConnection();

  if(m_frontLeftFail || m_frontRightFail || m_rearRightFail || m_rearLeftFail){
    if(m_frontLeftFail){
      SmartDashboard.putBoolean("Front left module connection", false);
      DataLogManager.log("Front Left swerve module disconnected");
    }
    if(m_frontRightFail){
      SmartDashboard.putBoolean("Front right module connection", false);
      DataLogManager.log("Front Right swerve module disconnected");
    }
    if(m_rearRightFail){
      SmartDashboard.putBoolean("Rear right module connection", false);
      DataLogManager.log("Rear Right swerve module disconnected");
    }
    if(m_rearLeftFail){
      SmartDashboard.putBoolean("Rear left module connection", false);
      DataLogManager.log("Rear left swerve module disconnected");
    }
    return true;
  }
  return false;
}

}
