package frc.robot;

import frc.robot.subsystems.Vision; 

public class Systems{

    private Vision vision;

    public Systems() {
        vision = new Vision();

        
    }

    public Vision getVision(){
        return vision;
    }


}