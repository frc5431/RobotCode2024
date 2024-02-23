package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5431.titan.core.joysticks.CommandSkyFlyController;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class DriverSkyflyController implements DriverController {
        private final CommandSkyFlyController controller;

    public DriverSkyflyController() {
        controller = new CommandSkyFlyController(1);
    }

    @Override
    public Trigger resetGyro() {
        return controller.rightSwitch();
    }

    @Override
    public Trigger stow() {
        return controller.backLeft();
    }

    @Override
    public double getLeftY() {
        return controller.getLeftY();
    }

    @Override
    public double getLeftX() {
        return controller.getLeftX();

    }

    @Override
    public double getRightX() {
        return controller.getRightX();
    }
    
}
