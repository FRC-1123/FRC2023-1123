// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmRaisePrepare extends CommandBase {
  private final ArmSubsystem m_armSubsystem;

  double m_upperArmPos;
  double m_lowerArmPos;
  double m_wristPos;
  boolean mediumScore;
  int time=0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmRaisePrepare(ArmSubsystem armed, double uAP, double lAP, double wP){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
  m_armSubsystem = armed;
  m_upperArmPos = uAP;
  m_lowerArmPos = lAP;
  m_wristPos = wP;
  }

  public ArmRaisePrepare(ArmSubsystem armed, double uAP, double lAP, double wP, boolean mediumScore){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
    m_armSubsystem = armed;
    m_upperArmPos = uAP;
    m_lowerArmPos = lAP;
    m_wristPos = wP;
    this.mediumScore = mediumScore;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mediumScore){
      m_armSubsystem.setUpperArmOutputRange(-.8, .3);
      m_armSubsystem.setLowerArmOutputRange(DriveConstants.m_lowerArmMinSpeed, 0.5);
    }
    else{
      m_armSubsystem.setUpperP(DriveConstants.defaultUpperArmP);
      m_armSubsystem.setUpperArmOutputRange(DriveConstants.m_upperArmMinSpeed, DriveConstants.m_upperArmMaxSpeed);
      m_armSubsystem.setLowerArmOutputRange(DriveConstants.m_lowerArmMinSpeed, DriveConstants.m_lowerArmMaxSpeed);
    }
    m_armSubsystem.setWristOutputRange(-.2, .2);

    time=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lowerArmMedianSet = -m_armSubsystem.getUpperArmPosition()/2;
    if(lowerArmMedianSet>m_lowerArmPos){
      lowerArmMedianSet = m_lowerArmPos;
    }
    // System.out.println(" upper arm position " + m_armSubsystem.getUpperArmPosition());
    // System.out.println("lower arm median set " +  lowerArmMedianSet);
    
    double upperArmMedianSet = m_upperArmPos + 50 - m_armSubsystem.getLowerArmPosition();
    if(upperArmMedianSet < m_upperArmPos){
      upperArmMedianSet = m_upperArmPos;
    }

    // System.out.println("upper arm median set" + upperArmMedianSet);
    // if(mediumScore){
    //   wristMedianSet = 90 - (m_armSubsystem.getUpperArmPosition());
    // }
    // else{
    //   wristMedianSet = 40 - (m_armSubsystem.getUpperArmPosition());
    // }
    // if(wristMedianSet > m_wristPos){
    //   wristMedianSet = m_wristPos;
    // }
    m_armSubsystem.setLowerPosition(lowerArmMedianSet);
    if(mediumScore){
      m_armSubsystem.setUpperPosition(m_upperArmPos);
    }
    else{
      m_armSubsystem.setUpperPosition(upperArmMedianSet);
    }
    if(!mediumScore){
      m_armSubsystem.setWristPosition(120);
    }
    else{
      m_armSubsystem.setWristPosition(m_wristPos);
    }
    // System.out.println("here lower " + lowerArmMedianSet + "upper " + upperArmMedianSet);
    // m_armSubsystem.setWristPosition(wristMedianSet);
    // if(time > 50){
    //   m_armSubsystem.setLowerPosition(m_lowerArmPos);
    // }
    // m_armSubsystem.setUpperPosition(m_upperArmPos);
    // m_armSubsystem.setWristPosition(m_wristPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setWristOutputRange(DriveConstants.m_wristMinSpeed, DriveConstants.m_wristMaxSpeed);
    m_armSubsystem.setUpperArmOutputRange(DriveConstants.m_upperArmMinSpeed, DriveConstants.m_upperArmMaxSpeed);
    if(mediumScore){
      m_armSubsystem.setUpperP(DriveConstants.defaultUpperArmP);
    }
    m_armSubsystem.setLowerPosition(m_lowerArmPos);
    m_armSubsystem.setUpperPosition(m_upperArmPos);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("upper arm delta  " + (m_armSubsystem.getUpperArmPosition() - m_upperArmPos));
    // System.out.println("lower arm delta  " + (m_armSubsystem.getLowerArmPosition() - m_lowerArmPos));
    // System.out.println("wrist arm delta  " + (m_armSubsystem.getWristPosition()- m_wristPos));
    if(mediumScore){
      if(Math.abs(m_armSubsystem.getUpperArmPosition() - m_upperArmPos)<25.0 && Math.abs(m_armSubsystem.getLowerArmPosition() - m_lowerArmPos)<8.0){
        System.out.println("in arm raise prepare medium finished");
        return true;
      }
    }
    else{
      if(Math.abs(m_armSubsystem.getUpperArmPosition() - m_upperArmPos)<15.0 && Math.abs(m_armSubsystem.getLowerArmPosition() - m_lowerArmPos)<5.0){
        System.out.println("in arm raise prepare finished");
        return true;
      }      
    }
  return false;
  }
}
