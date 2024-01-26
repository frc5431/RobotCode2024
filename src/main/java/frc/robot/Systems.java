package frc.robot;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision; 

public class Systems{

    private Vision vision;
    private Intake intake;
    private Angler intakeAngler;
    private Angler shooterAngler;
    private Drivebase drivebase;
    
    public Systems() {

        //vision = new Vision();
        intake = new Intake(new Spark(Constants.IntakeConstants.intakeId));
        // intakeAngler = new Angler(new CANSparkFlex(Constants.IntakeConstants.anglerId, MotorType.kBrushless), Constants.IntakeConstants.anglerConstants); //ready to be uncommented once ids are correct.
        // shooterAngler = new Angler(new CANSparkFlex(Constants.ShooterConstants.anglerId, MotorType.kBrushless), ShooterConstants.anglerConstants); //ready to be uncommented once ids are correct.


        drivebase = new Drivebase();
    }

    public Drivebase getDrivebase() {
        return drivebase;
    }

    public Vision getVision() {
        return vision;
    }

    public Intake getIntake() {
        return intake;
    }

    public Angler getIntakeAngler() {
        return intakeAngler;
    }

    public Angler getShooterAngler() {
        return shooterAngler;
    }


}