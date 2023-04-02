// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class ShootCubeSlow extends CommandBase {
  private final IntakeSubsystem intake;
  private boolean object_type;
  private boolean spit_out;
  private int time;

  double m_wristPos;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  // object_type: true = cone, false = cube ; spit_out: true = out, false = in
  public ShootCubeSlow(IntakeSubsystem intake){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0;
    intake.setCone(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time == 20){
      System.out.println("in intake in or out finished");
      return true;
    }
  return false;
  }
}
