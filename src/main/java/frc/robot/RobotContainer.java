// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunIntakeCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final Systems systems = new Systems();

  public RobotContainer() {

  }

  private void configureBindings() {
    m_operatorController.a().whileTrue(new RunIntakeCommand(true, systems.getIntake()));
    m_operatorController.y().whileTrue(new RunIntakeCommand(false, systems.getIntake()));
    m_operatorController.b().whileTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, systems.getAngler(), Constants.AnglerConstants.deployAngle.getRadians()));
    m_operatorController.x().whileTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.RETRACT, systems.getAngler(), Constants.AnglerConstants.retractAngle.getRadians()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
