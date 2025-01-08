package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class DriverXboxController implements DriverController {
    private final CommandXboxController controller;

    public DriverXboxController() {
        controller = new CommandXboxController(0);
    }


    @Override
    public Trigger resetGyro() {
        return controller.y();
    }

    @Override
    public Trigger stow() {
        return controller.leftTrigger();
    }

    @Override
    public double getLeftY() {
        return -controller.getLeftY();
    }

    @Override
    public double getLeftX() {
        return -controller.getLeftX();
    }

    @Override
    public double getRightX() {
        return controller.getRightX();
    }
    
    @Override
    public CommandXboxController temp_getController() {
        return controller;
    }
}
