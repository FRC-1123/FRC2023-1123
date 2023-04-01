// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class RunIntakeUntilStall extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
  private final IntakeSubsystem intake;
  double time = 0;
  boolean isCone;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeUntilStall(ArmSubsystem armed, IntakeSubsystem intake, boolean isCone){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armed, intake);
    m_armSubsystem = armed;
    this.intake = intake;
    this.isCone = isCone;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setPosition(0, 0, 10);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("intake Speed" + Math.abs(intake.getSpeed()));
    if(Timer.getFPGATimestamp()-time>0.1 && Math.abs(intake.getSpeed()) < 30){
      System.out.println("run intake until stall finished");
      return true;
    }
  return false;
  }
}
