// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmRaiseMediumTest extends CommandBase {
  private final ArmSubsystem m_armSubsystem;

  double m_upperArmPos;
  double m_lowerArmPos;
  double m_wristPos;
  int time=0;
  boolean stage1Done = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmRaiseMediumTest(ArmSubsystem armed, double uAP, double lAP, double wP){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
    m_armSubsystem = armed;
    m_upperArmPos = uAP;
    m_lowerArmPos = lAP;
    m_wristPos = wP;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage1Done = false;
    time=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wristMedianSet = 10;
    double lowerArmMedianSet = 0;
    double upperArmMedianSet = 0;
    if(stage1Done){
      // lowerArmMedianSet = -m_armSubsystem.getUpperArmPosition()/2;
      // if(lowerArmMedianSet>m_lowerArmPos){
        lowerArmMedianSet = m_lowerArmPos;
      // }
      // System.out.println(" upper arm position " + m_armSubsystem.getUpperArmPosition());
      // System.out.println("lower arm median set " +  lowerArmMedianSet);
      
      // upperArmMedianSet = m_upperArmPos + 50 - m_armSubsystem.getLowerArmPosition();
      // if(upperArmMedianSet < m_upperArmPos){
        upperArmMedianSet = m_upperArmPos;
      // }

      // System.out.println("upper arm median set" + upperArmMedianSet);
        wristMedianSet = m_wristPos;
    }
    else{
      upperArmMedianSet = m_upperArmPos;
      wristMedianSet = -m_armSubsystem.getUpperArmPosition() + 170;
      if(wristMedianSet < m_wristPos){
        wristMedianSet = m_wristPos;
      }
    }
    m_armSubsystem.setLowerPosition(lowerArmMedianSet);
    m_armSubsystem.setUpperPosition(upperArmMedianSet);
    m_armSubsystem.setWristPosition(wristMedianSet);
    if(Math.abs(m_armSubsystem.getWristPosition()- m_wristPos)<40.0 && Math.abs(m_armSubsystem.getUpperArmPosition() - m_upperArmPos)<15.0){
      stage1Done = true;
    }
    // if(time > 50){
    //   m_armSubsystem.setLowerPosition(m_lowerArmPos);
    // }
    // m_armSubsystem.setUpperPosition(m_upperArmPos);
    // m_armSubsystem.setWristPosition(m_wristPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("upper arm delta  " + (m_armSubsystem.getUpperArmPosition() - m_upperArmPos));
    // System.out.println("lower arm delta  " + (m_armSubsystem.getLowerArmPosition() - m_lowerArmPos));
    // System.out.println("wrist arm delta  " + (m_armSubsystem.getWristPosition()- m_wristPos));
    if(Math.abs(m_armSubsystem.getUpperArmPosition() - m_upperArmPos)<15.0 && Math.abs(m_armSubsystem.getLowerArmPosition() - m_lowerArmPos)<5.0
    && Math.abs(m_armSubsystem.getWristPosition()- m_wristPos)<40.0){
      System.out.println("in arm raise Medium test finished");
      return true;
    }
  return false;
  }
}
