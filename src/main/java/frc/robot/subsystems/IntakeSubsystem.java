package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;


public class IntakeSubsystem extends SubsystemBase {
    String scoreMode = "none";
    StringLogEntry scoreModeLog;
    // Initialize motors
    private final CANSparkMax motor = new CANSparkMax(DriveConstants.kIntakeMoterCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    public IntakeSubsystem(){
        motor.setSmartCurrentLimit(15);
        DataLog log = DataLogManager.getLog();
        scoreModeLog = new StringLogEntry(log, "/ScoreMode/");
    }

    public void setCone(double speed)
    {
        setMotor(-Math.abs(speed));
        scoreMode = "cone";
    }

    public void setCone()
    {
        setCone(1);
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

    public void setScoreModeNone(){
        scoreMode = "none";
    }
    public double getSpeed(){
        return encoder.getVelocity();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("speed of intake", getSpeed());
        scoreModeLog.append(scoreMode);
    }

    public boolean checkConnection(){
        int firmware = motor.getFirmwareVersion();
        if(firmware == 0){
            SmartDashboard.putBoolean("intake motor connection", false);
            DataLogManager.log("Intake rollers disconnected. firmware " + firmware);
            return true;
        }
        return false;
    }
}