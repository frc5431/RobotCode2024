package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverController {
    
    Trigger resetGyro();

    Trigger stow();

    double getLeftY();
    double getLeftX();
    double getRightX();

    CommandXboxController temp_getController();
}
