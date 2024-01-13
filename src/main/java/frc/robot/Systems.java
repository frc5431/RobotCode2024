package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision; 

public class Systems{

    private Vision vision;
    private Intake intake;
    private Angler angler;

    public Systems() {
        vision = new Vision();
        intake = new Intake(new Spark(0));
        angler = new Angler(new CANSparkFlex(0, MotorType.kBrushless));
    }

    public Vision getVision() {
        return vision;
    }

    public Intake getIntake () {
        return intake;
    }

    public Angler getAngler () {
        return angler;
    }
}