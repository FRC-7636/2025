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
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.commands.Auto.AutoDrive;
import frc.robot.commands.Auto.AutoDriveToBarge;
import frc.robot.commands.Auto.Reef;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import java.util.function.Supplier;

import javax.xml.crypto.KeySelector.Purpose;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;

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
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final XboxController test = new XboxController(1);
  private CommandXboxController testCtrl = new CommandXboxController(2);
  private final PS5Controller PS5 = new PS5Controller(3);
  
  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
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
  //                                                               () -> -MathUtil.applyDeadband(PS5.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  //                                                               () -> -MathUtil.applyDeadband(PS5.getLeftX(), OperatorConstants.DEADBAND),
  //                                                               () -> -MathUtil.applyDeadband(PS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
  //                                                               () -> PS5.getRawButton(4),
  //                                                               () -> PS5.getRawButton(2),
  //                                                               () -> PS5.getRawButton(1),
  //                                                               () -> PS5.getRawButton(3)
  //                                                               );        
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND),
                                                                () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                                                                () -> driverXbox.getHID().getYButtonPressed(),
                                                                () -> driverXbox.getHID().getAButtonPressed(),
                                                                () -> driverXbox.getHID().getXButtonPressed(),
                                                                () -> driverXbox.getHID().getBButtonPressed()
                                                                );        

                                                                /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  Supplier<ChassisSpeeds> fieldRelativeSpeeds = () -> new ChassisSpeeds(
    driverXbox.getLeftY() * -5,
    driverXbox.getLeftX() * -5, 
    driverXbox.getRightX() * -20
    );

  // Supplier<ChassisSpeeds> fieldRelativeSpeeds = () -> new ChassisSpeeds(
  //   test.getLeftY() * -1,
  //   test.getLeftX() * -1, 
  //   test.getRightX() * 1
  // );

  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation, right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(fieldRelativeSpeeds);

  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation, right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                                           .withControllerHeadingAxis(() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                                      () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)).headingWhile(true);
  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  private final AutoDriveToBarge autoDriveToBarge = new AutoDriveToBarge(drivebase, limelight);
  private final AutoDrive autoDrive = new AutoDrive(drivebase, limelight);
  private final Reef reef = new Reef(drivebase, limelight, vision);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
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
    // new JoystickButton(test, 1).whileTrue(autoDriveToBarge);
    new CommandXboxController(0).button(1).whileTrue(autoDriveToBarge);
    new CommandXboxController(0).button(2).whileTrue(autoDrive);

    new CommandXboxController(0).button(3).onTrue(reef);

    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    // if (Robot.isSimulation()) {
    //   driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    // }
    // if (DriverStation.isTest()) {
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    //   driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
    //   driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    //   driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    //   driverXbox.leftBumper().onTrue(Commands.none());
    //   driverXbox.rightBumper().onTrue(Commands.none());
    // } else {
    //   driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //   driverXbox.b().whileTrue(
    //       drivebase.driveToPose(
    //           new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
    //   driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
    //   driverXbox.start().whileTrue(Commands.none());
    //   driverXbox.back().whileTrue(Commands.none());
    //   driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverXbox.rightBumper().onTrue(Commands.none());
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Auto");
    // return new AutoDriveToBarge(drivebase, limelight);
    return new AutoDrive(drivebase, limelight);
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
