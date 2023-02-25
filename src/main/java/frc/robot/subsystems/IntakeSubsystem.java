package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;


public class IntakeSubsystem extends SubsystemBase {
    String scoreMode = "none";
    // Initialize motors
    private final CANSparkMax motor = new CANSparkMax(DriveConstants.kIntakeMoterCanId, MotorType.kBrushless);
    public IntakeSubsystem(){
    }

    public void setCone(double speed)
    {
        setMotor(-Math.abs(speed));
        scoreMode = "cone";
    }

    public void setCone()
    {
        setCone(0.5);
    }

    public void setCube(double speed)
    {
        setMotor(Math.abs(speed));
        scoreMode = "cube";
    }

    public void setCube()
    {
        setCube(0.5);
    }

    public void setMotor(double value){
        motor.set(value);
    }

    public void setStop()
    {
        motor.stopMotor();
    }

    public String getScoreMode(){
        return scoreMode;
    }
}