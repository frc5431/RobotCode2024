package frc.robot;

import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
<<<<<<< Updated upstream
=======
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision; 
>>>>>>> Stashed changes

public class Systems{

    private Vision vision;
    private Intake intake;
    private Angler angler;
    private Drivebase drivebase;

    private Spark intakeVortex;
    private CANSparkFlex anglerFlex;
    
    public Systems() {
<<<<<<< Updated upstream

        

        drivebase = new Drivebase(Constants.Drivebase.DrivetrainConstants, Constants.Drivebase.frontLeft,
            Constants.Drivebase.frontRight, Constants.Drivebase.BackLeft, Constants.Drivebase.BackRight);
        vision = new Vision();
        intake = new Intake(intakeVortex);
        angler = new Angler(anglerFlex);
        
=======
        // vision = new Vision();
        // intake = new Intake(new Spark(0));
        // angler = new Angler(new CANSparkFlex(0, MotorType.kBrushless));
        drivebase = new Drivebase();
    }

    public Drivebase getDrivebase() {
        return drivebase;
>>>>>>> Stashed changes
    }

    public Vision getVision() {
        return vision;
    }

    public Intake getIntake() {
        return intake;
    }

    public Angler getAngler() {
        return angler;
    }

    public Drivebase getDrivebase() {
        return drivebase;
    }

}