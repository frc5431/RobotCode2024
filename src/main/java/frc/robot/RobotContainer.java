// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunIntakeCommand.DirectionEnum;
import frc.robot.subsystems.Drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {


  private final CommandXboxController driver =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final Systems systems = new Systems();
    public final Drivebase drivebase = systems.getDrivebase();


  

  public RobotContainer() {
    configureBindings();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        } else {
            return (value + deadband) / (1.0 - deadband);
        }
    } else {
        return 0.0;
    }
}

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.15);

    // More sensitive at smaller speeds
    double newValue = Math.pow(value, 2);

    // Copy the sign to the new value
    newValue = Math.copySign(newValue, value);

    return newValue;
}

  private void configureBindings() {
    driver.setDeadzone(0.15);

     drivebase.setDefaultCommand(new DefaultDriveCommand(
            systems,
            (Supplier<Pair<Double, Double>>) () -> {
                double inX = -driver.getLeftY(); // swap intended
                double inY = -driver.getLeftX();
                double mag = Math.hypot(inX, inY);
                double theta = Math.atan2(inY, inX);
                return Pair.of(modifyAxis(mag) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND, theta);
            },
            () -> modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));





    operator.a().whileTrue(new RunIntakeCommand(DirectionEnum.INTAKE, systems.getIntake()));
    operator.y().whileTrue(new RunIntakeCommand(DirectionEnum.OUTTAKE, systems.getIntake()));
    operator.b().whileTrue(new RunAnglerCommand(AnglerModes.DEPLOY, systems.getAngler()));
    operator.x().whileTrue(new RunAnglerCommand(AnglerModes.RETRACT, systems.getAngler()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
