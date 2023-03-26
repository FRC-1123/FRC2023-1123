// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
public class TestingAutoBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_subsystem;
  int time = 0;
  public PIDController m_rollController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestingAutoBalance(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    m_rollController = new PIDController(.5,0,0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = m_subsystem.getPitch();
    if(Timer.getMatchTime() < 0.125){
      m_subsystem.setX();
    }
    else if(Math.abs(pitch)< 2.5){
      m_subsystem.setX();
  
      
      } else if(Math.abs(pitch) < 11){
        m_subsystem.drive(m_rollController.calculate(pitch,0)/5,0,0,true);
      } else {
        m_subsystem.drive(-Math.signum(Units.degreesToRadians(pitch)),0,0,true);
      }

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setX();
    System.out.println("Command Canceled");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
