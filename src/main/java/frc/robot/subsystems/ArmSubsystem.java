package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    CANSparkMax m_lowerArmMotor;
    CANSparkMax m_upperArmMotor;

    RelativeEncoder m_lowerArmEncoder;
    RelativeEncoder m_upperArmEncoder;

    SparkMaxPIDController m_lowerPIDController;
    SparkMaxPIDController m_upperPIDController;

    public ArmSubsystem(){
        //TODO: add a wrist motor
        //sets the values of the upper and lower arm motors
        m_lowerArmMotor = new CANSparkMax(20, MotorType.kBrushless);
        m_upperArmMotor = new CANSparkMax(21, MotorType.kBrushless);
    
        //Restores factory defaults of the spark maxes
        m_lowerArmMotor.restoreFactoryDefaults();
        m_upperArmMotor.restoreFactoryDefaults();
        
        //Gets the encoder values from the motors
        m_lowerArmEncoder = m_lowerArmMotor.getEncoder();
        m_upperArmEncoder = m_upperArmMotor.getEncoder();

        //Gets the PID controller from the motors
        m_lowerPIDController = m_lowerArmMotor.getPIDController();
        m_upperPIDController = m_upperArmMotor.getPIDController();

        //Sets the feedback devices for the PID controllers
        m_lowerPIDController.setFeedbackDevice(m_lowerArmEncoder);
        m_upperPIDController.setFeedbackDevice(m_lowerArmEncoder);//TODO: something is wrong here

        //Sets the position and velocity factors of the encoders
        m_lowerArmEncoder.setPositionConversionFactor(1/210);
        m_lowerArmEncoder.setVelocityConversionFactor(1/210);
        m_upperArmEncoder.setPositionConversionFactor(1/280);
        m_upperArmEncoder.setVelocityConversionFactor(1/280);
        
        //Sets the PID values of the controller
        m_lowerPIDController.setP(.1);
        m_lowerPIDController.setI(0);
        m_lowerPIDController.setD(0);
        m_upperPIDController.setP(.1);
        m_upperPIDController.setI(0);
        m_upperPIDController.setD(0);

        m_lowerArmMotor.setOpenLoopRampRate(.2);
        m_upperArmMotor.setOpenLoopRampRate(.2);
        
        m_lowerArmMotor.burnFlash();
        m_upperArmMotor.burnFlash();               
    }

    public void stopMotors(){
        m_lowerPIDController.setReference(0, CANSparkMax.ControlType.kVoltage);
        m_upperPIDController.setReference(0, CANSparkMax.ControlType.kVoltage);
    }

    public void setPosition(double lowerArmPos, double upperArmPos){
        m_lowerPIDController.setReference(lowerArmPos, CANSparkMax.ControlType.kPosition);
        m_upperPIDController.setReference(upperArmPos, CANSparkMax.ControlType.kPosition);
    }

    public void setVoltage(double lowerArmVolt, double upperArmVolt){
        m_lowerPIDController.setReference(lowerArmVolt, CANSparkMax.ControlType.kVoltage);
        m_upperPIDController.setReference(upperArmVolt, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Lower Arm Position", m_lowerArmEncoder.getPosition());
        SmartDashboard.putNumber("Upper Arm position", m_upperArmEncoder.getPosition());

        SmartDashboard.putNumber("Lower Arm Voltage", m_lowerArmMotor.getBusVoltage());
        SmartDashboard.putNumber("Upper Arm Voltage", m_upperArmMotor.getBusVoltage());
        //TODO: there is another way to control the motors I want to do
    
    }

    public void setPid(double upperP, double upperI, double upperD, double lowerP, double lowerI, double lowerD){
        m_upperPIDController.setP(upperP);
        m_upperPIDController.setI(upperI);
        m_upperPIDController.setD(upperD);
        m_lowerPIDController.setP(lowerP);
        m_lowerPIDController.setI(lowerI);
        m_lowerPIDController.setD(lowerD);
    }
}
