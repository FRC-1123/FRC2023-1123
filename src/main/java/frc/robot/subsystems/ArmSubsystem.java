package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    CANSparkMax m_lowerArmMotor;
    CANSparkMax m_upperArmMotor;
    CANSparkMax m_wristMotor;

    RelativeEncoder m_lowerArmEncoder;
    RelativeEncoder m_upperArmEncoder;
    AbsoluteEncoder m_wristEncoder;

    SparkMaxPIDController m_lowerPIDController;
    SparkMaxPIDController m_upperPIDController;
    SparkMaxPIDController m_wristPIDController;

    double lowerArmSetpoint = 0;
    double upperArmSetpoint = 0;
    double wristSetpoint = 0;

    boolean lowerArmPosEnabled = false;
    boolean upperArmPosEnabled = false;
    boolean wristPosEnabled = false;

    double wristArbFF = 0.35;
    double upperArmArbFF = 0.2;
    double lowerArmArbFF = 0.2;

    double wristToDegrees = 1;
    double upperToDegrees = 1;
    double lowerToDegrees = 1;
    
    public ArmSubsystem(){
        //sets the values of the upper and lower arm motors
        m_lowerArmMotor = new CANSparkMax(20, MotorType.kBrushless);
        m_upperArmMotor = new CANSparkMax(21, MotorType.kBrushless);
        m_wristMotor = new CANSparkMax(22, MotorType.kBrushed);
    
        //Restores factory defaults of the spark maxes
        m_lowerArmMotor.restoreFactoryDefaults();
        m_upperArmMotor.restoreFactoryDefaults();
        m_wristMotor.restoreFactoryDefaults();

        m_lowerArmMotor.setIdleMode(IdleMode.kBrake);
        m_upperArmMotor.setIdleMode(IdleMode.kBrake);
        m_wristMotor.setIdleMode(IdleMode.kBrake);

        
        //Gets the encoder values from the motors
        m_lowerArmEncoder = m_lowerArmMotor.getEncoder();
        m_upperArmEncoder = m_upperArmMotor.getEncoder();
        m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

        //Gets the PID controller from the motors
        m_lowerPIDController = m_lowerArmMotor.getPIDController();
        m_upperPIDController = m_upperArmMotor.getPIDController();
        m_wristPIDController = m_wristMotor.getPIDController();

        //Sets the feedback devices for the PID controllers
        m_lowerPIDController.setFeedbackDevice(m_lowerArmEncoder);
        m_upperPIDController.setFeedbackDevice(m_upperArmEncoder);
        m_wristPIDController.setFeedbackDevice(m_wristEncoder);

        //Sets the position and velocity factors of the encoders
        m_lowerArmEncoder.setPositionConversionFactor(360/210);
        m_lowerArmEncoder.setVelocityConversionFactor(1/210);
        m_upperArmEncoder.setPositionConversionFactor(360/280);
        m_upperArmEncoder.setVelocityConversionFactor(1/280);

        m_wristEncoder.setPositionConversionFactor(360);

        m_lowerPIDController.setOutputRange(-0.5,0.5);
        m_upperPIDController.setOutputRange(-0.5,0.5);
        m_wristPIDController.setOutputRange(-0.7,0.7);

        m_lowerPIDController.setPositionPIDWrappingEnabled(false);
        m_upperPIDController.setPositionPIDWrappingEnabled(false);
        m_wristPIDController.setPositionPIDWrappingEnabled(false);
        
        //Sets the PID values of the controller
        m_lowerPIDController.setP(.1);
        m_lowerPIDController.setI(0);
        m_lowerPIDController.setD(0);
        m_upperPIDController.setP(.1);
        m_upperPIDController.setI(0);
        m_upperPIDController.setD(0);
        m_wristPIDController.setP(.1);
        m_wristPIDController.setI(0);
        m_wristPIDController.setD(0);

        m_lowerArmMotor.setOpenLoopRampRate(.1);
        m_upperArmMotor.setOpenLoopRampRate(.1);
        m_wristMotor.setOpenLoopRampRate(.1);
        
        m_lowerArmMotor.setClosedLoopRampRate(.1);
        m_upperArmMotor.setClosedLoopRampRate(.1);
        m_wristMotor.setClosedLoopRampRate(.1);

        // m_lowerArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
        // m_lowerArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);
        
        m_lowerArmMotor.burnFlash();
        m_upperArmMotor.burnFlash();               
    }

    public void stopMotors(){
        m_lowerPIDController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        m_upperPIDController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        m_wristPIDController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        lowerArmPosEnabled = false;
        upperArmPosEnabled = false;
        wristPosEnabled = false;
    }

    public void setPosition(double lowerArmPos, double upperArmPos, double wristPos){
        lowerArmSetpoint = lowerArmPos;
        upperArmSetpoint = upperArmPos;
        wristSetpoint = wristPos;

        lowerArmPosEnabled = true;
        upperArmPosEnabled = true;
        wristPosEnabled = true; 
    }

    public void setLowerPosition(double lowerArmPos){
        lowerArmSetpoint = lowerArmPos;
        lowerArmPosEnabled = true;
    }

    public void setUpperPosition(double upperArmPos){
        upperArmSetpoint = upperArmPos;
        upperArmPosEnabled = true;
    }

    public void setWristPosition(double wristPos){
        wristSetpoint = wristPos;
        wristPosEnabled = true;
    }

    public void setVoltage(double lowerArmVolt, double upperArmVolt, double wristVolt){
        m_lowerPIDController.setReference(lowerArmVolt, CANSparkMax.ControlType.kDutyCycle);
        m_upperPIDController.setReference(upperArmVolt, CANSparkMax.ControlType.kDutyCycle);
        m_wristPIDController.setReference(wristVolt, CANSparkMax.ControlType.kDutyCycle);
        lowerArmPosEnabled = false;
        upperArmPosEnabled = false;
        wristPosEnabled = false;
    }

    public void setLowerVoltage(double lowerArmVolt){
        m_lowerPIDController.setReference(lowerArmVolt, CANSparkMax.ControlType.kDutyCycle);
        lowerArmPosEnabled = false;
    }

    public void setUpperVoltage(double upperArmVolt){
        m_upperPIDController.setReference(upperArmVolt, CANSparkMax.ControlType.kDutyCycle);
        upperArmPosEnabled = false;
    }

    public void setWristVoltage(double wristVolt){
        m_wristPIDController.setReference(wristVolt, CANSparkMax.ControlType.kDutyCycle);
        wristPosEnabled = false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Lower Arm Position", m_lowerArmEncoder.getPosition());
        SmartDashboard.putNumber("Upper Arm position", m_upperArmEncoder.getPosition());

        SmartDashboard.putNumber("Lower Arm Voltage", m_lowerArmMotor.getBusVoltage());
        SmartDashboard.putNumber("Upper Arm Voltage", m_upperArmMotor.getBusVoltage());

        if(wristPosEnabled){
            if(getWristPosition() < 15 && wristSetpoint < 15){

            }
            else{
                double arbFeedForward = -Math.cos(Math.toRadians(m_wristEncoder.getPosition()/wristToDegrees))*wristArbFF;
                m_wristPIDController.setReference(wristSetpoint, CANSparkMax.ControlType.kPosition, 0, arbFeedForward);
            }
        }

        if(upperArmPosEnabled){
            if(getUpperArmPosition() < 15 && upperArmSetpoint < 15){
                m_upperPIDController.setReference(0, CANSparkMax.ControlType.kVoltage);
            }
            else{
                double arbFeedForward = Math.cos(Math.toRadians(m_upperArmEncoder.getPosition()/upperToDegrees))*upperArmArbFF;
                m_upperPIDController.setReference(upperArmSetpoint, CANSparkMax.ControlType.kPosition, 0, arbFeedForward);
            }
        }

        if(lowerArmPosEnabled){
            if(getLowerArmPosition() < 15 && lowerArmSetpoint < 15){
                m_upperPIDController.setReference(0, CANSparkMax.ControlType.kVoltage);
            }
            else{
                double arbFeedForward = Math.cos(Math.toRadians(m_lowerArmEncoder.getPosition()/lowerToDegrees))*lowerArmArbFF;
                m_lowerPIDController.setReference(lowerArmSetpoint, CANSparkMax.ControlType.kPosition, 0, arbFeedForward);
            }
        }
    
    }

    public void setPid(double upperP, double upperI, double upperD, double lowerP, double lowerI,
     double lowerD, double wristP, double wristI, double wristD){
        m_upperPIDController.setP(upperP);
        m_upperPIDController.setI(upperI);
        m_upperPIDController.setD(upperD);
        m_lowerPIDController.setP(lowerP);
        m_lowerPIDController.setI(lowerI);
        m_lowerPIDController.setD(lowerD);
        m_wristPIDController.setP(wristP);
        m_wristPIDController.setI(wristI);
        m_wristPIDController.setD(wristD);
    }

    public double getLowerArmPosition(){
        return m_lowerArmEncoder.getPosition();
    }
    public double getUpperArmPosition(){
        return m_upperArmEncoder.getPosition();
    }
    public double getWristPosition(){
        return m_wristEncoder.getPosition();
    }
}
