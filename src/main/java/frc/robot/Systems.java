package frc.robot;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision; 

public class Systems{

    private Intake intake;

    private Vision vision;

    public Systems() {
        vision = new Vision();
        intake = new Intake();
        
    }

    public Vision getVision(){
        return vision;
    }
     public Intake getIntake(){
        return intake;
     }
    }
