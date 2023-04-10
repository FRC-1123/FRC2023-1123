// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class FlipIntakeThenBack extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
  private final IntakeSubsystem intake;
  int time = 0;
  boolean isCone;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlipIntakeThenBack(ArmSubsystem armed, IntakeSubsystem intake, boolean isCone){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed);
    m_armSubsystem = armed;
    this.intake = intake;
    this.isCone = isCone;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0;
    m_armSubsystem.setWristPosition(DriveConstants.m_wristFoldedBackwards);
    m_armSubsystem.setUpperPosition(DriveConstants.m_upperArmFoldedBackwards);
    if(isCone){
      intake.setCone();
    }
    else{
      intake.setCube();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time++;
    if(time > 25 && Math.abs(intake.getSpeed()) < 500){
      m_armSubsystem.setWristPosition(DriveConstants.m_WristIn);
      m_armSubsystem.setUpperPosition(0);
      if(isCone){
        intake.setCone(0.2);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_armSubsystem.getWristPosition()- DriveConstants.m_WristIn) < 10.0 && Math.abs(m_armSubsystem.getUpperArmPosition() - 0) < 10 && time> 50){

      return true;
    }
  return false;
  }
}
