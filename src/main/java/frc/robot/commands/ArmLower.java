// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmLower extends CommandBase {
  private final ArmSubsystem m_armSubsystem;

  double m_upperArmPos;
  double m_lowerArmPos;
  double m_wristPos;
  boolean slowMode = false;
  boolean fastMode = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmLower(ArmSubsystem armed, double uAP, double lAP, double wP){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
  m_armSubsystem = armed;
  m_upperArmPos = uAP;
  m_lowerArmPos = lAP;
  m_wristPos = wP;
  }

  public ArmLower(ArmSubsystem armed, double uAP, double lAP, double wP, boolean slowMode){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
  m_armSubsystem = armed;
  m_upperArmPos = uAP;
  m_lowerArmPos = lAP;
  m_wristPos = wP;
  this.slowMode = slowMode;
  }

  public ArmLower(Boolean fastMode, ArmSubsystem armed, double uAP, double lAP, double wP){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
  m_armSubsystem = armed;
  m_upperArmPos = uAP;
  m_lowerArmPos = lAP;
  m_wristPos = wP;
  this.fastMode = fastMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(slowMode){
      m_armSubsystem.setUpperArmOutputRange(-.3, .3);
    }
    else if(fastMode){
      m_armSubsystem.setUpperArmOutputRange(DriveConstants.m_upperArmMinSpeed, 0.7);
    }
      else{
      m_armSubsystem.setLowerArmOutputRange(DriveConstants.m_lowerArmMinSpeed, DriveConstants.m_lowerArmMaxSpeed);
      m_armSubsystem.setUpperArmOutputRange(DriveConstants.m_upperArmMinSpeed, DriveConstants.m_upperArmMaxSpeed);
      m_armSubsystem.setWristOutputRange(DriveConstants.m_wristMinSpeed, DriveConstants.m_wristMaxSpeed);
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upperArmMedianSet = -m_armSubsystem.getLowerArmPosition()*1.6-2;
    if(upperArmMedianSet > m_upperArmPos){
      upperArmMedianSet = m_upperArmPos;
    }

    //System.out.println("upper Arm median set" + upperArmMedianSet);

    double lowerArmMedianSet = m_lowerArmPos;
    double wristMedianSet = m_wristPos;

    
    
    m_armSubsystem.setLowerPosition(lowerArmMedianSet);
    m_armSubsystem.setUpperPosition(upperArmMedianSet);
    m_armSubsystem.setWristPosition(wristMedianSet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(slowMode || fastMode){
      m_armSubsystem.setUpperArmOutputRange(DriveConstants.m_upperArmMinSpeed,DriveConstants.m_upperArmMaxSpeed);
    }
    // m_armSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_armSubsystem.getLowerArmPosition() - m_lowerArmPos)<5.0 && Math.abs(m_armSubsystem.getUpperArmPosition() - m_upperArmPos)<5.0
    && Math.abs(m_armSubsystem.getWristPosition()- m_wristPos)<10.0){
      //System.out.println("in finished");
      return true;
    }
  return false;
  }
}
