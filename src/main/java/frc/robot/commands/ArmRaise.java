// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmRaise extends CommandBase {
  private final ArmSubsystem m_armSubsystem;

  double m_upperArmPos;
  double m_lowerArmPos;
  double m_wristPos;
  boolean mediumScore;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmRaise(ArmSubsystem armed, double uAP, double lAP, double wP){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
  m_armSubsystem = armed;
  m_upperArmPos = uAP;
  m_lowerArmPos = lAP;
  m_wristPos = wP;
  }

  public ArmRaise(ArmSubsystem armed, double uAP, double lAP, double wP, boolean mediumScore){
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
  public void initialize() {}

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
    double wristMedianSet = 0;
    if(mediumScore){
      wristMedianSet = 90 - (m_armSubsystem.getUpperArmPosition());
    }
    else{
      wristMedianSet = 90 - (m_armSubsystem.getUpperArmPosition()/1.5);
    }
    if(wristMedianSet > m_wristPos){
      wristMedianSet = m_wristPos;
    }
    m_armSubsystem.setLowerPosition(lowerArmMedianSet);
    m_armSubsystem.setUpperPosition(upperArmMedianSet);
    m_armSubsystem.setWristPosition(wristMedianSet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_armSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("upper arm delta  " + (m_armSubsystem.getUpperArmPosition() - m_upperArmPos));
    System.out.println("lower arm delta  " + (m_armSubsystem.getLowerArmPosition() - m_lowerArmPos));
    System.out.println("wrist arm delta  " + (m_armSubsystem.getWristPosition()- m_wristPos));
    if(Math.abs(m_armSubsystem.getUpperArmPosition() - m_upperArmPos)<15.0 && Math.abs(m_armSubsystem.getLowerArmPosition() - m_lowerArmPos)<5.0
    && Math.abs(m_armSubsystem.getWristPosition()- m_wristPos)<78.0){
      System.out.println("in finished");
      return true;
    }
  return false;
  }
}
