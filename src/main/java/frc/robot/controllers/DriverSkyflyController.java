package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5431.titan.core.joysticks.CommandSkyFlyController;

public class DriverSkyflyController implements DriverController {
        private final CommandSkyFlyController controller;

    public DriverSkyflyController() {
        controller = new CommandSkyFlyController(0);
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
        return -controller.getLeftX();
    }

    @Override
    public double getLeftX() {
        return -controller.getLeftY();

    }

    @Override
    public double getRightX() {
        return controller.getRightX();
    }

    @Override
    public CommandXboxController temp_getController() {
        return null;
    }
    
}
