
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.misc.Logger;

public class DriveCommands extends Command {

    public Command DefaultDriveCommand(){
        
       public DefaultDriveCommand() {

       }

       @Override
       public void 




    }

}

// private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
        //                                                             // driving in open loop
        // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        // private void configureBindings() {
        // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
        //                                                                                         // negative Y (forward)
        //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     ));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain
        //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));