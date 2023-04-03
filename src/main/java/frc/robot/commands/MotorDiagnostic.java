package frc.robot.commands;

 
import java.text.DecimalFormat;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DiagnosticConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MAXSwerveModule;

public class MotorDiagnostic extends CommandBase{
    private final static DecimalFormat decfor = new DecimalFormat("0.00");
    DriveSubsystem drive_system;
    MAXSwerveModule swerve_system;
    ArmSubsystem arm_system;
    IntakeSubsystem intake_system;

    /////////////////////////
    ///SWERVE DRIVE VALUES///
    /////////////////////////

    // final wheel positions
    double frontLeftP;
    double frontRightP;
    double rearLeftP;
    double rearRightP;

    // wheel positions at start
    double frontLeftZ;
    double frontRightZ;
    double rearLeftZ;
    double rearRightZ;

    // wheel speedsat 0.5 (slowish)
    double frontLeftSS;
    double frontRightSS;
    double rearLeftSS;
    double rearRightSS;

    // wheel speeds at 1.0 (fast)
    double frontLeftSF;
    double frontRightSF;
    double rearLeftSF;
    double rearRightSF;

    // first wheel angles
    Rotation2d frontLeftR1;
    Rotation2d frontRightR1;
    Rotation2d rearLeftR1;
    Rotation2d rearRightR1;

    // final wheel angles
    Rotation2d frontLeftR2;
    Rotation2d frontRightR2;
    Rotation2d rearLeftR2;
    Rotation2d rearRightR2;

    // speed percent differences
    double frontLeftSSD;
    double frontRightSSD;
    double rearLeftSSD;
    double rearRightSSD;

    double frontLeftSFD;
    double frontRightSFD;
    double rearLeftSFD;
    double rearRightSFD;

    // position percent defferences
    double frontLeftPD;
    double frontRightPD;
    double rearLeftPD;
    double rearRightPD;

    double frontLeftR1D;
    double frontRightR1D;
    double rearLeftR1D;
    double rearRightR1D;

    double frontLeftR2D;
    double frontRightR2D;
    double rearLeftR2D;
    double rearRightR2D;

    //////////////////
    ////ARM VALUES////
    //////////////////

    // first extension arm positions
    double lowerArmP;
    double upperArmP;
    double wristP;

    // final arm positions
    double lowerArmF;
    double upperArmF;
    double wristF;

    // inital arm positions
    double lowerArmZ;
    double upperArmZ;
    double wristZ;

    // roller speeds
    double rollerSpeedOut;
    double rollerSpeedIn;

    // position percent differences
    double lowerArmPD;
    double upperArmPD;
    double wristPD;

    double lowerArmFD;
    double upperArmFD;
    double wristFD;

    double rollerSpeedInD;
    double rollerSpeedOutD;


    /////////////////
    ///CHECK CASES///
    /////////////////

    public boolean wheelHalfSpeed;
    public boolean wheelFullSpeed;
    public boolean wheelRotation1;
    public boolean wheelRotation2;
    public boolean wheelDistance;

    public boolean firstArmCheck;
    public boolean finalArmCheck;
    public boolean rollerSpeedCheck;

    ///////////
    ///OTHER///
    ///////////

    int time;
    public boolean hasRunDiagnostic;



    public MotorDiagnostic(DriveSubsystem drive_system, ArmSubsystem arm_system, IntakeSubsystem intake_system){
        this.drive_system = drive_system;
        this.arm_system = arm_system;
        this.intake_system = intake_system;
        addRequirements(drive_system, arm_system, intake_system);

        hasRunDiagnostic = false;
        wheelHalfSpeed = false;
        wheelFullSpeed = false;
        wheelRotation1 = false;
        wheelRotation2 = false;
        wheelDistance = false;
        firstArmCheck = false;
        finalArmCheck = false;
        rollerSpeedCheck = false;

    }

    @Override
    public void initialize(){
        // record inital wheel positions
        frontLeftZ = Math.abs(drive_system.m_frontLeft.getPosition().distanceMeters);
        frontRightZ = Math.abs(drive_system.m_frontRight.getPosition().distanceMeters);
        rearLeftZ = Math.abs(drive_system.m_rearLeft.getPosition().distanceMeters);
        rearRightZ = Math.abs(drive_system.m_rearRight.getPosition().distanceMeters);

        time = 0;

        System.out.println("Starting motor recording.");
    }

    @Override
    public void execute(){
        if(time == 0){
            drive_system.drive(0.5, 0, 0, false);
        }
        if(time == 300){
            // record wheel speeds at 0.5 x
            frontLeftSS = drive_system.m_frontLeft.getState().speedMetersPerSecond;
            frontRightSS = drive_system.m_frontRight.getState().speedMetersPerSecond;
            rearLeftSS = drive_system.m_rearLeft.getState().speedMetersPerSecond;
            rearRightSS = drive_system.m_rearRight.getState().speedMetersPerSecond;

            drive_system.drive(1, 0, 0, false);
        }
        if(time == 500){
            // record wheel speeds at 1.0 x
            frontLeftSF = drive_system.m_frontLeft.getState().speedMetersPerSecond;
            frontRightSF = drive_system.m_frontRight.getState().speedMetersPerSecond;
            rearLeftSF = drive_system.m_rearLeft.getState().speedMetersPerSecond;
            rearRightSF = drive_system.m_rearRight.getState().speedMetersPerSecond;

            // record final wheel positions
            frontLeftP = drive_system.m_frontLeft.getPosition().distanceMeters;
            frontRightP = drive_system.m_frontRight.getPosition().distanceMeters;
            rearLeftP = drive_system.m_rearLeft.getPosition().distanceMeters;
            rearRightP = drive_system.m_rearRight.getPosition().distanceMeters;

            drive_system.drive(0, 0.1, 0, false);
        }
        if(time == 550){
            // record rotation of wheels
            frontLeftR1 = drive_system.m_frontLeft.getState().angle;
            frontRightR1 = drive_system.m_frontRight.getState().angle;
            rearLeftR1 = drive_system.m_rearLeft.getState().angle;
            rearRightR1 = drive_system.m_rearRight.getState().angle;

            drive_system.drive(0.1,0,0,false); 
        }
        if(time == 600){
            // record rotation of wheels
            frontLeftR2 = drive_system.m_frontLeft.getState().angle;
            frontRightR2 = drive_system.m_frontRight.getState().angle;
            rearLeftR2 = drive_system.m_rearLeft.getState().angle;
            rearRightR2 = drive_system.m_rearRight.getState().angle;

            drive_system.drive(0,0,0,false);
        }
        if(time == 650){
            // record inital arm positions
            lowerArmZ = arm_system.getLowerArmPosition();
            upperArmZ = arm_system.getUpperArmPosition();
            wristZ = arm_system.getWristPosition();

            arm_system.setPosition(15, -30, 45);
        }
        if(time == 800){
            // record arm positions
            lowerArmP = arm_system.getLowerArmPosition();
            upperArmP = arm_system.getUpperArmPosition();
            wristP = arm_system.getWristPosition();

            arm_system.setPosition(0,0,10);
        }
        if(time == 950){
            // record final arm positions
            lowerArmF = arm_system.getLowerArmPosition();
            upperArmF = arm_system.getUpperArmPosition();
            wristF = arm_system.getWristPosition();

            intake_system.setCone();
        }
        if(time == 1000){
            // record intake speed
            rollerSpeedIn = intake_system.getSpeed();
            intake_system.setCube();
        }
        if(time == 1050){
            rollerSpeedOut = intake_system.getSpeed();
            intake_system.setStop();
        }
        
        time++;
    }

    @Override
    public void end(boolean interrupted){
        drive_system.drive(0,0,0,false);

        System.out.println("Recording Done. Analyzing Data...");
        // prepare report
        frontLeftSFD = getPercentDif(frontLeftSF, DiagnosticConstants.normalWheelSpeedFullSpeed);
        frontRightSFD = getPercentDif(frontRightSF, DiagnosticConstants.normalWheelSpeedFullSpeed);
        rearLeftSFD = getPercentDif(rearLeftSF, DiagnosticConstants.normalWheelSpeedFullSpeed);
        rearRightSFD = getPercentDif(rearRightSF, DiagnosticConstants.normalWheelSpeedFullSpeed);

        frontLeftSSD = getPercentDif(frontLeftSS, DiagnosticConstants.normalWheelSpeedHalfSpeed);
        frontRightSSD = getPercentDif(frontRightSS, DiagnosticConstants.normalWheelSpeedHalfSpeed);
        rearLeftSSD = getPercentDif(rearLeftSS, DiagnosticConstants.normalWheelSpeedHalfSpeed);
        rearRightSSD = getPercentDif(rearRightSS, DiagnosticConstants.normalWheelSpeedHalfSpeed);

        frontLeftPD = getPercentDif(getWheelPosition(frontLeftZ, frontLeftP), DiagnosticConstants.normalWheelPosition);
        frontRightPD = getPercentDif(getWheelPosition(frontRightZ, frontRightP), DiagnosticConstants.normalWheelPosition);
        rearLeftPD = getPercentDif(getWheelPosition(rearLeftZ, rearLeftP), DiagnosticConstants.normalWheelPosition);
        rearRightPD = getPercentDif(getWheelPosition(rearRightZ, rearRightP), DiagnosticConstants.normalWheelPosition);

        frontLeftR1D = getPercentDif(frontLeftR1.getDegrees(), DiagnosticConstants.normalRotationPosition1);
        frontRightR1D = getPercentDif(frontRightR1.getDegrees(), DiagnosticConstants.normalRotationPosition1);
        rearLeftR1D = getPercentDif(rearLeftR1.getDegrees(), DiagnosticConstants.normalRotationPosition1);
        rearRightR1D = getPercentDif(rearRightR1.getDegrees(), DiagnosticConstants.normalRotationPosition1);

        frontLeftR2D = getPercentDif(frontLeftR2.getDegrees(), DiagnosticConstants.normalRotationPosition2);
        frontRightR2D = getPercentDif(frontRightR2.getDegrees(), DiagnosticConstants.normalRotationPosition2);
        rearLeftR2D = getPercentDif(rearLeftR2.getDegrees(), DiagnosticConstants.normalRotationPosition2);
        rearRightR2D = getPercentDif(rearRightR2.getDegrees(), DiagnosticConstants.normalRotationPosition2);

        // arm percent differences
        lowerArmPD = getPercentDif(lowerArmP, DiagnosticConstants.lowerArmTestPosition);
        upperArmPD = getPercentDif(upperArmP, DiagnosticConstants.upperArmTestPosition);
        wristPD = getPercentDif(wristP, DiagnosticConstants.wristTestPosition);

        lowerArmFD = getPercentDif(lowerArmF, 0);
        upperArmFD = getPercentDif(upperArmF, 0);
        wristFD = getPercentDif(wristF, 10);

        rollerSpeedInD = getPercentDif(rollerSpeedIn, DiagnosticConstants.rollerSpeedIn);
        rollerSpeedOutD = getPercentDif(rollerSpeedOutD, DiagnosticConstants.rollerSpeedOut);

        // do percent difference checks
        wheelFullSpeed = checkDifference(frontLeftSFD, frontRightSFD, rearLeftSFD, rearRightSFD, DiagnosticConstants.percentDifferenceLimit);
        wheelHalfSpeed = checkDifference(frontLeftSSD, frontRightSSD, rearLeftSSD, rearRightSSD, DiagnosticConstants.percentDifferenceLimit);
        wheelRotation1 = checkDifference(frontLeftR1D, frontRightR1D, rearLeftR1D, rearRightR1D, DiagnosticConstants.percentDifferenceLimit);
        wheelRotation2 = checkDifference(frontLeftR2D, frontRightR2D, rearLeftR2D, rearRightR2D, DiagnosticConstants.percentDifferenceLimit);

        wheelDistance = checkDifference(frontLeftPD, frontRightPD, rearLeftPD, rearRightPD, DiagnosticConstants.percentDifferenceLimit);

        firstArmCheck = checkDifference(lowerArmPD, upperArmPD, wristPD, DiagnosticConstants.percentDifferenceLimit);
        finalArmCheck = checkDifference(lowerArmFD, upperArmFD, wristFD, DiagnosticConstants.percentDifferenceLimit);
        rollerSpeedCheck = checkDifference(rollerSpeedInD, rollerSpeedOutD, 0, DiagnosticConstants.percentDifferenceLimit);


        // print report
        printStartReport();

        printSection("Wheel Half-Speeds");
        printStat("Front Left Wheel Speed", frontLeftSSD);
        printStat("Front Right Wheel Speed", frontRightSSD);
        printStat("Rear Left Wheel Speed", rearLeftSSD);
        printStat("Rear Right Wheel Speed", rearRightSSD);
        printStat("Wheel Half-Speeds Good? ", wheelHalfSpeed);

        printSection("wheel full-speeds");
        printStat("Front Left Wheel Speed", frontLeftSFD);
        printStat("Front Right Wheel Speed", frontRightSFD);
        printStat("Rear Left Wheel Speed", rearLeftSFD);
        printStat("Rear Right Wheel Speed", rearRightSFD);
        printStat("Wheel Full-Speeds Good? ", wheelFullSpeed);

        printSection("Final wheel distance");
        printStat("Front Left Wheel Distance", frontLeftPD);
        printStat("Front Right Wheel Distance", frontRightPD);
        printStat("Rear Left Wheel Distance", rearLeftPD);
        printStat("Rear Right Wheel Distance", rearRightPD);
        printStat("Wheel Distance Good? ", wheelDistance);

        printSection("Wheel rotations horizontal");
        printStat("Front Left Wheel Rotation", frontLeftR1D);
        printStat("Front Right Wheel Rotation", frontRightR1D);
        printStat("Rear Left Wheel Rotation", rearLeftR1D);
        printStat("Rear Right Wheel Rotation", rearRightR1D);
        printStat("Wheel Rotations Good? ", wheelRotation1);

        printSection("wheel rotations vertical");
        printStat("Front Left Wheel Rotation", frontLeftR2D);
        printStat("Front Right Wheel Rotation", frontRightR2D);
        printStat("Rear Left Wheel Rotation", rearLeftR2D);
        printStat("Rear Right Wheel Rotation", rearRightR2D);
        printStat("Wheel Rotations Good? ", wheelRotation2);

        printSection("arm positions out");
        printStat("Lower Arm Position", lowerArmPD);
        printStat("Upper Arm Position", upperArmPD);
        printStat("Wrist Position", wristPD);
        printStat("Arm Positions Good? ", firstArmCheck);

        printSection("arm positions in");
        printStat("Lower Arm Position", lowerArmFD);
        printStat("Upper Arm Position", upperArmFD);
        printStat("Wrist Position", wristFD);
        printStat("Arm Positions Good? ", finalArmCheck);

        printSection("Roller speeds");
        printStat("Roller Speed In", rollerSpeedInD);
        printStat("Roller Speed Out", rollerSpeedOutD);
        printStat("Roller Speeds Good? ", rollerSpeedCheck);

        printEndReport();

        // post values to smartdashboard
        hasRunDiagnostic = true;
        // SmartDashboard.putBoolean("Wheel Half-Speeds Good?", wheelHalfSpeed);
        // SmartDashboard.putBoolean("Wheel Full-Speeds Good?", wheelFullSpeed);
        // SmartDashboard.putBoolean("Wheel Distance Good?", wheelDistance);
        // SmartDashboard.putBoolean("Wheels can go Horizontal?", wheelRotation1);
        // SmartDashboard.putBoolean("Wheels can go Vertical?", wheelRotation2);
        // SmartDashboard.putBoolean("Arm can Extend?", firstArmCheck);
        // SmartDashboard.putBoolean("Arm can Fold?", finalArmCheck);
        // SmartDashboard.putBoolean("Roller Speeds Good?", rollerSpeedCheck); 

    }

    @Override
    public boolean isFinished(){
        if(time == 1100){
            return true;
        }
        else{
            return false;
        }
    }

    private double getPercentDif(double testValue, double standardValue){
        testValue = Math.abs(testValue - standardValue);
        if(standardValue == 0){
            return testValue * 100;
        }
        else{
            testValue = testValue / standardValue;
        }
        return testValue * 100;
    }

    private double getWheelPosition(double start, double end){
        return end - Math.abs(start);
    }

    private boolean checkDifference(double difference1, double difference2, double difference3, double difference4, double compare_pecent){
        if(difference1 > compare_pecent && difference2 > compare_pecent && difference3 > compare_pecent && difference4 > compare_pecent){
            return false;
        }
        else{
            return true;
        }
    }

    private boolean checkDifference(double difference1, double difference2, double difference3, double compare_pecent){
        if(difference1 > compare_pecent && difference2 > compare_pecent && difference3 > compare_pecent){
            return false;
        }
        else{
            return true;
        }
    }

    private void printStartReport(){
        System.out.println("########################");
        System.out.println("");
        System.out.println("#### MOTOR DIAGNOSTIC REPORT ####");
        System.out.println("");
        System.out.println("SHOWS PERCENT DIFFERENCES BETWEEN TESTED");
        System.out.println("AND NORMAL MOTOR VALUES.");
        System.out.println("");
        System.out.println("VERY HIGH PERCENT DIFFERENCES MAY INDICATE");
        System.out.println("MOTOR ENCODER FAILURE.");
        System.out.println("");
    }

    private void printEndReport(){
        System.out.println("");
        System.out.println("#### END OF MOTOR DIAGNOSTIC REPORT ####");
        System.out.println("");

        System.out.println("########################");
    }

    private void printStat(String name, double stat){
        System.out.println(name+": "+decfor.format(stat)+"%");
        System.out.println("");
    }

    private void printStat(String name, boolean stat){
        System.out.println(name+": "+stat);
        System.out.println("");
    }

    private void printSection(String message){
        System.out.print("##### "+message.toUpperCase()+" #####");
        System.out.println("");
    }
}
