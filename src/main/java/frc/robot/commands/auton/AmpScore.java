package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class AmpScore extends SequentialCommandGroup {
    
  public AmpScore(Shooter shooter, Intake intake) {

    addCommands(
      Commands.parallel(
      shooter.runShooterCommand(ShooterModes.AmpShot),
      new WaitCommand(1).andThen(RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE).withTimeout(0.3)
        ) 
      )
    );
  }
}
