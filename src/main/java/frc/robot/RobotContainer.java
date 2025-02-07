// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.commands.Auto.AutoDrive;
import frc.robot.commands.Auto.AutoDriveToBarge;
import frc.robot.commands.Auto.REEF2;
import frc.robot.commands.Auto.Reef;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

import java.io.File;
import java.util.function.Supplier;

import javax.xml.crypto.KeySelector.Purpose;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */

public class RobotContainer{
  // Controllers
  // private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final XboxController test = new XboxController(1);
  private CommandXboxController testCtrl = new CommandXboxController(2);
  private final PS5Controller PS5 = new PS5Controller(3);
  
  // Subsystems
  // private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
  private final Vision vision = new Vision();
  private final Algae algae = new Algae();
  private final Climber climber = new Climber();
  private final Coral coral = new Coral();
  private final Elevator elevator = new Elevator();
  private final limelight limelight = new limelight();
  
  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive.
  // left stick controls translation, right stick controls the rotational velocity, buttons are quick rotation positions to different ways to face.
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings.
  // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
  //                                                               () -> -MathUtil.applyDeadband(test.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  //                                                               () -> -MathUtil.applyDeadband(test.getLeftX(), OperatorConstants.DEADBAND),
  //                                                               () -> -MathUtil.applyDeadband(test.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
  //                                                               () -> test.getRawButtonPressed(4),
  //                                                               () -> test.getRawButton(2),
  //                                                               () -> test.getRawButton(1),
  //                                                               () -> test.getRawButton(3)
  //                                                               );

  // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                               () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //                                                               () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND),
    //                                                               () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    //                                                               () -> driverXbox.getHID().getYButtonPressed(),
    //                                                               () -> driverXbox.getHID().getAButtonPressed(),
    //                                                               () -> driverXbox.getHID().getXButtonPressed(),
    //                                                               () -> driverXbox.getHID().getBButtonPressed()
    //                                                               );        
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //     () -> test.getLeftY() * -1,
  //     () -> test.getLeftX() * -1)
  //     .withControllerRotationAxis(test::getRightX)
  //     .deadband(OperatorConstants.DEADBAND)
  //     .scaleTranslation(0.8)
  //     .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(test::getRightX,
  //     test::getRightY)
  //     .headingWhile(true);

  // Supplier<ChassisSpeeds> fieldRelativeSpeeds = () -> new ChassisSpeeds(
  //   test.getLeftY() * -5,
  //   test.getLeftX() * -5, 
  //   test.getRightX() * -15
  //   );

  // Supplier<ChassisSpeeds> fieldRelativeSpeeds = () -> new ChassisSpeeds(
  //   test.getLeftY() * 0,
  //   test.getLeftX() * 0, 
  //   test.getRightX() * 0
  //   );

  // Supplier<ChassisSpeeds> fieldRelativeSpeeds = () -> new ChassisSpeeds(
  //   test.getLeftY() * -1,
  //   test.getLeftX() * -1, 
  //   test.getRightX() * 10
  // )

  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation, right stick controls the desired angle NOT angular rotation
  
  // Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(fieldRelativeSpeeds);

  // Applies deadbands and inverts because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation, right stick controls the angular velocity of the robot
  
  // Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  // SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //     () -> -test.getLeftY(),
  //     () -> -test.getLeftX())
  //     .withControllerRotationAxis(() -> test.getRawAxis(2))
  //     .deadband(OperatorConstants.DEADBAND)
  //     .scaleTranslation(0.8)
  //     .allianceRelativeControl(true);
  // // Derive the heading axis with math!
  // SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                                          //  .withControllerHeadingAxis(() -> Math.sin(test.getRawAxis(2) 
                                          //  * Math.PI) * (Math.PI * 2),
                                                                      // () -> Math.cos(test.getRawAxis(2) * Math.PI) * (Math.PI * 2)).headingWhile(true);
  // Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  // Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  // private final AutoDriveToBarge autoDriveToBarge = new AutoDriveToBarge(drivebase, limelight);
  // private final AutoDrive autoDrive = new AutoDrive(drivebase, limelight, vision);
  // private final Reef reef = new Reef(drivebase, limelight, vision);
  // private final REEF2 reef2 = new REEF2(drivebase, limelight);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // new JoystickButton(test, 2).whileTrue(autoDrive);
    // new JoystickButton(test, 1).whileTrue(autoDriveToBarge);

    // new JoystickButton(test, 1).whileTrue(new InstantCommand((algae::ShuShu) -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));
    new JoystickButton(test, 1).onTrue(new InstantCommand(algae::ShuShu));
    new JoystickButton(test, 2).onTrue(new InstantCommand(algae::BomBom));
    new JoystickButton(test, 3).whileTrue(new InstantCommand(algae::CC)).onFalse(new InstantCommand(algae::Stop));
    new JoystickButton(test, 4).whileTrue(new InstantCommand(algae::TT)).onFalse(new InstantCommand(algae::Stop));
    new JoystickButton(test, 5).onTrue(new InstantCommand(algae::ShuBom));
    new JoystickButton(test, 6).whileTrue(new InstantCommand(algae::Shu)).onFalse(new InstantCommand(algae::Stop));
    new JoystickButton(test, 7).whileTrue(new InstantCommand(algae::Bom)).onFalse(new InstantCommand(algae::Stop));
    new JoystickButton(test, 8).onTrue(new InstantCommand(algae::Stop));
    // new JoystickButton(test, 5).whileTrue(new InstantCommand(algae::ShuC));
    // new JoystickButton(test, 6).whileTrue(new InstantCommand(algae::BomT));

    // (Condition) ? Return-On-True :
    //  Return-on-False
    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    // if (Robot.isSimulation()) {
      // driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    // }
    // if (DriverStation.isTest()) {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    //   driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    //   driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    //   driverXbox.leftBumper().onTrue(Commands.none());
    //   driverXbox.rightBumper().onTrue(Commands.none());
    // } else {
      // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());
      // new JoystickButton(test, 5).onTrue(drivebase.driveToPose(new Pose2d(new Translation2d(14, 4), Rotation2d.fromDegrees(0))));
      // new JoystickButton(test, 5).whileTrue(Commands.runOnce( () -> drivebase.resetOdometry(new Pose2d(12, 6, new Rotation2d(0)))));
        // drivebase.driveToPose(
      // new JoystickButton(test, 6).whileTrue(drivebase.driveToPose(((vision.getTagPose()))));
      // new JoystickButton(test, 6).whileTrue(drivebase.driveToPose(new Pose2d(15.464, 7.352, Units.degreesToRadians(3.180))));
      // new JoystickButton(PS5, 0).whileTrue(drivebase.driveToPose(new Pose2d(15, 7, 3.18)));
      // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
    // return autoDriveToBarge;
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }
}
