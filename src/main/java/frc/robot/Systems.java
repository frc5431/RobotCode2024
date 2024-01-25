package frc.robot;

import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision; 

public class Systems{

    private Vision vision;
    private Intake intake;
    private Angler angler;
    private Drivebase drivebase;

    private Spark intakeVortex;
    private CANSparkFlex anglerFlex;
    
    public Systems() {

        //vision = new Vision();
        intake = new Intake(intakeVortex);
        //angler = new Angler(anglerFlex);

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

    public Angler getAngler() {
        return angler;
    }


}