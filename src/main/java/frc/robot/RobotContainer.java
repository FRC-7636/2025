// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto_Cmd.AutoDriveOneMeter;
import frc.robot.commands.Auto_Cmd.AutoDriveToBarge;
import frc.robot.commands.Auto_Cmd.AutoPath;
import frc.robot.commands.Auto_Cmd.AutoToReef;
import frc.robot.commands.Auto_Cmd.DriveToCoralStation;
import frc.robot.commands.Auto_Cmd.DriveToReef18;
import frc.robot.commands.Auto_Cmd.REEF2;
import frc.robot.commands.Auto_Cmd.Reef;
import frc.robot.commands.Auto_Cmd.test;
import frc.robot.commands.Auto_Cmd.CMD.CoralSation;
import frc.robot.commands.Auto_Cmd.CMD.REEFL1;
import frc.robot.commands.Auto_Cmd.CMD.REEFL2;
import frc.robot.commands.Auto_Cmd.CMD.REEFL3;
import frc.robot.commands.Auto_Cmd.CMD.REEFL4;
import frc.robot.commands.Group_Cmd.Algae_Intake;
import frc.robot.commands.Group_Cmd.Algae_Release;
import frc.robot.commands.Group_Cmd.Coral_Station;
import frc.robot.commands.Group_Cmd.RL1;
import frc.robot.commands.Group_Cmd.RL2;
import frc.robot.commands.Group_Cmd.RL3;
import frc.robot.commands.Group_Cmd.RL4;
import frc.robot.commands.Group_Cmd.SetZero;
import frc.robot.commands.Single_Cmd.Climb;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.Candle;

import java.io.File;
import java.util.PrimitiveIterator;
import java.util.function.Supplier;

import javax.xml.crypto.KeySelector.Purpose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final XboxController Drive_Ctrl = new XboxController(1);
  private final XboxController Ctrl = new XboxController(2);
  // private CommandXboxController testCtrl = new CommandXboxController(2);
  private final PS5Controller PS5 = new PS5Controller(3);
  
  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/kraken"));
  private final Vision vision = new Vision();
  private final Algae algae = new Algae();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();
  private final Coral coral = new Coral();
  private final Elevator elevator = new Elevator();
  private final limelight limelight = new limelight();
  private final Candle candle = new Candle();

  // Single CMD
  private final Climb CMD_Climb = new Climb(climber);

  // Group CMD
  private final Algae_Intake CMD_Algae_Intake = new Algae_Intake(algae);
  private final Algae_Release CMD_Algae_Release = new Algae_Release(algae);
  private final Coral_Station CMD_Coral_Station = new Coral_Station(arm, coral, elevator);
  private final RL1 CMD_RL1 = new RL1(arm, coral, elevator);
  private final RL2 CMD_RL2 = new RL2(arm, coral, elevator);
  private final RL3 CMD_RL3 = new RL3(arm, coral, elevator);
  private final RL4 CMD_RL4 = new RL4(arm, coral, elevator);
  private final SetZero CMD_SetZero = new SetZero(algae, arm, candle, climber, coral, elevator);

  // Auto CMD
  private final CoralSation CMD_CoralSation = new CoralSation(arm, coral, elevator);
  private final REEFL1 CMD_Reefl1 = new REEFL1(arm, coral, elevator);
  private final REEFL2 CMD_Reefl2 = new REEFL2(arm, coral, elevator);
  private final REEFL3 CMD_Reefl3 = new REEFL3(arm, coral, elevator);
  private final REEFL4 CMD_Reefl4 = new REEFL4(arm, coral, elevator);

  private final AutoDriveOneMeter CMD_AutoDriveOneMeter = new AutoDriveOneMeter(drivebase);
  private final AutoDriveToBarge CMD_AutoDriveToBarge = new AutoDriveToBarge(drivebase, limelight, elevator);
  private final AutoPath CMD_AutoPath = new AutoPath(drivebase, drivebase.getSwerveDrive());
  private final AutoToReef CMD_AutoToReef = new AutoToReef(drivebase);
  private final DriveToCoralStation CMD_DriveToCoralStation = new DriveToCoralStation(drivebase, limelight);
  private final DriveToReef18 CMD_DriveToReef18= new DriveToReef18(drivebase, limelight, vision);
  private final test test_ = new test(drivebase, drivebase.getSwerveDrive());
  // private final Reef reef = new Reef(drivebase, limelight, vision);
  // private final REEF2 reef2 = new REEF2(drivebase, limelight);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private SendableChooser<Command> getAutoChooser = new SendableChooser<>();

  private final static File[] pathFileList = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive.
  // left stick controls translation, right stick controls the rotational velocity, buttons are quick rotation positions to different ways to face.
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings.
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                () -> -MathUtil.applyDeadband(Drive_Ctrl.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                                                () -> -MathUtil.applyDeadband(Drive_Ctrl.getLeftX(), OperatorConstants.DEADBAND),
                                                                () -> -MathUtil.applyDeadband(Drive_Ctrl.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                                                                () -> Drive_Ctrl.getRawButtonPressed(4),
                                                                () -> Drive_Ctrl.getRawButton(2),
                                                                () -> Drive_Ctrl.getRawButton(1),
                                                                () -> Drive_Ctrl.getRawButton(3)
                                                                );

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
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> Drive_Ctrl.getLeftY() * -1,
      () -> Drive_Ctrl.getLeftX() * -1)
      .withControllerRotationAxis(Drive_Ctrl::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
                                                                                              Drive_Ctrl::getRightX,
                                                                                              Drive_Ctrl::getRightY)
                                                                                              .headingWhile(true);

  // Supplier<ChassisSpeeds> fieldRelativeSpeeds = () -> new ChassisSpeeds(
  //                                                                       Drive_Ctrl.getLeftY() * -3,
  //                                                                       Drive_Ctrl.getLeftX() * -3, 
  //                                                                       Drive_Ctrl.getRightX() * -10
  //                                                                       );

  Supplier<ChassisSpeeds> fieldRelativeSpeeds = () -> new ChassisSpeeds(
                                                                        Drive_Ctrl.getLeftY() * 0,
                                                                        Drive_Ctrl.getLeftX() * 0, 
                                                                        Drive_Ctrl.getRightX() * 0
                                                                        );

  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation, right stick controls the desired angle NOT angular rotation
  
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(fieldRelativeSpeeds);
  // Applies deadbands and inverts because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation, right stick controls the angular velocity of the robot
  
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> -Drive_Ctrl.getLeftY(),
                                                                    () -> -Drive_Ctrl.getLeftX())
                                                                    .withControllerRotationAxis(() -> Drive_Ctrl.getRawAxis(2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                                           .withControllerHeadingAxis(() -> Math.sin(Drive_Ctrl.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                                      () -> Math.cos(Drive_Ctrl.getRawAxis(2) * Math.PI) * (Math.PI * 2)).headingWhile(true);
  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("CoralStation", CMD_CoralSation);
    NamedCommands.registerCommand("REEFL1", CMD_Reefl1);
    NamedCommands.registerCommand("REEFL2", CMD_Reefl2);
    NamedCommands.registerCommand("REEFL3", CMD_Reefl3);
    NamedCommands.registerCommand("REEFL4", CMD_Reefl4);


    NamedCommands.registerCommand("RL1", CMD_RL1);
    NamedCommands.registerCommand("RL2", CMD_RL2);
    NamedCommands.registerCommand("RL3", CMD_RL3);
    NamedCommands.registerCommand("RL4", CMD_RL4);
    NamedCommands.registerCommand("Algae_Intake", CMD_Algae_Intake);
    NamedCommands.registerCommand("Algae_Release", CMD_Algae_Release);
    NamedCommands.registerCommand("Climb", CMD_Climb);

    getAutoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", getAutoChooser);

    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
    // setMotorBrake(false);
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
  private void configureBindings(){
    new JoystickButton(Drive_Ctrl, 2).onTrue(CMD_AutoPath);
    new JoystickButton(Drive_Ctrl, 1).onTrue(CMD_AutoDriveToBarge);
    new JoystickButton(Drive_Ctrl, 3).onTrue(CMD_AutoToReef);
    new JoystickButton(Drive_Ctrl, 4).whileTrue(CMD_AutoDriveOneMeter);
    new JoystickButton(Drive_Ctrl, 5).whileTrue(CMD_DriveToReef18);
    new JoystickButton(Drive_Ctrl, 6).onTrue(new InstantCommand(drivebase::setPOS));

    // new JoystickButton(Drive_Ctrl, 1).onTrue(new InstantCommand(algae::Intake_out).alongWith(new WaitCommand(0.5).andThen(new InstantCommand(algae::Stop))));
    // new JoystickButton(Drive_Ctrl, 2).onTrue(new InstantCommand(algae::Intake_back).alongWith(new WaitCommand(0.5).andThen(new InstantCommand(algae::Stop))));
    // new JoystickButton(Drive_Ctrl, 3).whileTrue(new InstantCommand(algae::suck)).onFalse(new InstantCommand(algae::Stop));
    // new JoystickButton(Drive_Ctrl, 4).whileTrue(new InstantCommand(algae::shoot)).onFalse(new InstantCommand(algae::Stop));

    // Algae
    new JoystickButton(Ctrl, 1).whileTrue(new InstantCommand(arm::Arm_DOWN)).onFalse(new InstantCommand(arm::Arm_Stop));
    new JoystickButton(Ctrl, 2).whileTrue(new InstantCommand(arm::Arm_UP)).onFalse(new InstantCommand(arm::Arm_Stop));

    // Reef
    // new JoystickButton(Ctrl, 3).onTrue(CMD_RL1);
    // new JoystickButton(Ctrl, 4).onTrue(CMD_RL2);
    // new JoystickButton(Ctrl, 5).onTrue(CMD_RL3);

    new JoystickButton(Ctrl, 3).whileTrue(new InstantCommand(elevator::ELE_Down)).onFalse(new InstantCommand(elevator::ELE_Stop));
    new JoystickButton(Ctrl, 4).whileTrue(new InstantCommand(elevator::ELE_Up)).onFalse(new InstantCommand(elevator::ELE_Stop));
    // new JoystickButton(Ctrl, 7).onTrue(new InstantCommand(elevator::ELE_Floor));
    
    new JoystickButton(Ctrl, 5).onTrue(new InstantCommand(coral::Coral_Suck)).onFalse(new InstantCommand(coral::Coral_Stop));
    // Coral 
    new JoystickButton(Ctrl, 6).whileTrue(new InstantCommand(coral::Coral_Shoot)).onFalse(new InstantCommand(coral::Coral_Stop));

    // new JoystickButton(test, 2).onTrue(new InstantCommand(algae::Intake_back).alongWith(new WaitCommand(0.5).andThen(new InstantCommand(algae::Stop))));
    
    new POVButton(Ctrl, 0).whileTrue(new InstantCommand(arm::Arm_UP)).onFalse(new InstantCommand(arm::Arm_Stop));
    new POVButton(Ctrl, 180).whileTrue(new InstantCommand(arm::Arm_DOWN)).onFalse(new InstantCommand(arm::Arm_Stop));

    // new Trigger(Ctrl.axisGreaterThan(3, .1, null)).onTrue(CMD_DriveToReef18).onFalse(CMD_DriveToCoralStation);

    /*
    // Driver
    new JoystickButton(PS5, 1).onTrue(new InstantCommand(algae::Intake_out).alongWith(new WaitCommand(0.5).andThen(new InstantCommand(algae::Stop))));
    new JoystickButton(PS5, 2).onTrue(new InstantCommand(algae::Intake_back).alongWith(new WaitCommand(0.5).andThen(new InstantCommand(algae::Stop))));

    new JoystickButton(PS5, 3).onTrue(CMD_Climb);
    new JoystickButton(PS5, 4).onTrue(CMD_SetZero);

    new JoystickButton(PS5, 5).onTrue(new InstantCommand(coral::Coral_Suck)).onFalse(new InstantCommand(coral::Coral_Stop));
    new JoystickButton(PS5, 6).whileTrue(new InstantCommand(coral::Coral_Shoot)).onFalse(new InstantCommand(coral::Coral_Stop));

    new JoystickButton(PS5, 7).onTrue(CMD_Algae_Intake);
    new JoystickButton(PS5, 8).onTrue(CMD_Algae_Intake);

    new JoystickButton(PS5, 14).onTrue(new InstantCommand(drivebase::setHead));

    new POVButton(PS5, 0).onTrue(CMD_RL1);
    new POVButton(PS5, 90).onTrue(CMD_RL2);
    new POVButton(PS5, 180).onTrue(CMD_RL3);
    new POVButton(PS5, 270).onTrue(CMD_RL4);

    // Assist
    new JoystickButton(Ctrl, 1).onTrue(new InstantCommand(arm::Arm_Coral_RL1));    // 1
    new JoystickButton(Ctrl, 2).onTrue(new InstantCommand(arm::Arm_Coral_RL2));    // 2
    new JoystickButton(Ctrl, 3).onTrue(new InstantCommand(arm::Arm_Coral_RL4));    // 3
    new JoystickButton(Ctrl, 4).onTrue(new InstantCommand(arm::Arm_Coral_Sation)); // 4

    new JoystickButton(Ctrl, 8).onTrue(new InstantCommand(arm::Arm_Coral_DOWN));   // 8

    new JoystickButton(Ctrl, 5).whileTrue(new InstantCommand(elevator::ELE_Down)).onFalse(new InstantCommand(elevator::ELE_Stop));
    new JoystickButton(Ctrl, 6).whileTrue(new InstantCommand(elevator::ELE_Up)).onFalse(new InstantCommand(elevator::ELE_Stop));
    new JoystickButton(Ctrl, 7).onTrue(new InstantCommand(elevator::ELE_Floor));
  
    */

    // (Condition) ? Return-On-True :

    //  Return-on-False
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

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
      // driverXbox.leftBumper().onTrue(Commands.none());
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
    // return null;
    return CMD_DriveToReef18;
            // .andThen(
            //   Commands.runOnce(() -> drivebase.drive(new ChassisSpeeds(0, 0, 0)), 
            //   drivebase
            //   ));
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
