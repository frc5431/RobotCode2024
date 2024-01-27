package frc.robot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.AnglerConstants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;


public class Systems {

  private Vision vision;
  private Intake intake;
  private Angler intakeAngler;
  private Angler shooterAngler;
  private Drivebase drivebase;

  private AnglerConstants intakeAnglerConst;

  private CANSparkBase intakeAngler550;

  public Systems() {
    intakeAnglerConst = Constants.IntakeConstants.anglerConstants;
    intakeAngler550 = new CANSparkMax(Constants.IntakeConstants.anglerId, MotorType.kBrushless);

    drivebase = new Drivebase();

    // vision = new Vision();
    intake = new Intake(new CANSparkFlex(Constants.IntakeConstants.intakeId, MotorType.kBrushless));
    intakeAngler = new Angler(intakeAngler550, intakeAnglerConst , "intake angler");
    // CANSparkFlex(Constants.IntakeConstants.anglerId, MotorType.kBrushless),
    // Constants.IntakeConstants.anglerConstants, "Intake Angler"); //ready to be
    // uncommented once ids are correct.
    // shooterAngler = new Angler(new
    // CANSparkFlex(Constants.ShooterConstants.anglerId, MotorType.kBrushless),
    // ShooterConstants.anglerConstants, "Shooter Angler"); //ready to be
    // uncommented once ids are correct.

  }

  public Drivebase getDrivebase() {
    return drivebase;
  }

  public Vision getVision() {
    return vision;
  }

  public Intake getIntake() {
    return intake;
  }

  public Angler getIntakeAngler() {
    return intakeAngler;
  }

  public Angler getShooterAngler() {
    return shooterAngler;
  }
}
