package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

    /** Creates a new ExampleSubsystem. */
    WPI_TalonFX falconMotor;
    public ExampleSubsystem() {
        falconMotor = new WPI_TalonFX(10);
        falconMotor.config_kP(0, 0.1);
    }
    
    public void setPosition(double position) {
        falconMotor.set(ControlMode.Position, position);
    }
    
    public void setPercent(double percentOutput){
        falconMotor.set(ControlMode.PercentOutput, percentOutput);
    }
       
    public void setVelocity(double velocity){
        falconMotor.set(ControlMode.Velocity, velocity);
    }


}
