package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AIMConstants;


public class ConeIntakePrototype extends SubsystemBase {
    // Initialize motors
    private final CANSparkMax topRoller = new CANSparkMax(AIMConstants.kTopRollerCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax bottomRoller = new CANSparkMax(AIMConstants.kBottomRollerCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    public ConeIntakePrototype(){
        topRoller.setInverted(true);
    }

    public void setIntake()
    {
        topRoller.set(0.5);
        bottomRoller.set(0.5);
    }

    public void setExpell()
    {
        topRoller.set(-0.5);
        bottomRoller.set(-0.5);
    }

    public void setStop()
    {
        topRoller.stopMotor();
        bottomRoller.stopMotor();
    }
}