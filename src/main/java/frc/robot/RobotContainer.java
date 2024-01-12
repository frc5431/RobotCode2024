// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunIntakeCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {

  private final Systems systems = new Systems();

  private final CommandXboxController m_OperatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
  }
  private void configureMethods(){
    m_OperatorController.a().whileTrue(new RunIntakeCommand(true,systems.getIntake()));
    m_OperatorController.b().whileTrue(new RunIntakeCommand(false,systems.getIntake()));
  }
  public Command getAutonomousCommand() {
    return null;
  }

}
