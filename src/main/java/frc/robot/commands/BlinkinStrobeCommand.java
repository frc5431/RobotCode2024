package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.leds.BlinkinPattern;

public class BlinkinStrobeCommand extends Command {
    int curActionFrames = 0;
    boolean isOn = false;
    final int frequency = 5;

    Blinkin blinkin;
    BlinkinPattern pattern;

    public BlinkinStrobeCommand(Blinkin blinkin, BlinkinPattern pattern) {
        this.blinkin = blinkin;
        this.pattern = pattern;
        addRequirements(blinkin);
    }   

    @Override
    public void initialize() {
        curActionFrames = 0;
        isOn = true;
        blinkin.set(pattern);
    }

    @Override
    public void execute() {
        curActionFrames++;
        SmartDashboard.putNumber("strob", curActionFrames);

        if(curActionFrames > frequency) {
            curActionFrames = 0;
            isOn = !isOn;
            if(isOn) {
                blinkin.set(pattern);
            } else {
                blinkin.set(BlinkinPattern.BLACK);
            }
        }
    }

 

}
