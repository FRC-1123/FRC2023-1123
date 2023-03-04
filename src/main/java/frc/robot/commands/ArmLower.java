// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmLower extends CommandBase {
  private final ArmSubsystem m_armSubsystem;

  double m_upperArmPos;
  double m_lowerArmPos;
  double m_wristPos;
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upperArmMedianSet = -m_armSubsystem.getLowerArmPosition() - 6;
    if(upperArmMedianSet > m_upperArmPos){
      upperArmMedianSet = m_upperArmPos;
    }

    System.out.println("upper Arm median set" + upperArmMedianSet);

    double lowerArmMedianSet = m_lowerArmPos;
    double wristMedianSet = m_wristPos;

    
    
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
    if(Math.abs(m_armSubsystem.getUpperArmPosition() - m_upperArmPos)<5.0 && Math.abs(m_armSubsystem.getLowerArmPosition() - m_lowerArmPos)<5.0
    && Math.abs(m_armSubsystem.getWristPosition()- m_wristPos)<5.0){
      System.out.println("in finished");
      return true;
    }
  return false;
  }
}
