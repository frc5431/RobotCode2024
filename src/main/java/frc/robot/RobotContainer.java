// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5431.titan.core.joysticks.CommandXboxController;
// import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
