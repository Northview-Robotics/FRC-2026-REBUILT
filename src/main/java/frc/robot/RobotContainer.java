// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeAngle;
import frc.robot.subsystems.IntakeRollers;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;


  private final CommandXboxController driverCtrl = new CommandXboxController(0);
  private final CommandXboxController operatorCtrl = new CommandXboxController(1);

  private final IntakeRollers intakeRollers = new IntakeRollers(Constants.IntakeRollersConstants.intakeRollerID, Constants.IntakeRollersConstants.gearRatio);
  private final IntakeAngle intakeAngle = new IntakeAngle(Constants.IntakePivotConstants.intakePivotID, Constants.IntakePivotConstants.gearRatio);
  private final Indexer indexer = new Indexer(Constants.IndexerConstants.indexerID, Constants.IndexerConstants.gearRatio);

  private File directory = new File(Filesystem.getDeployDirectory(),"swerve2");
  private final Drive drivetrain = new Drive(directory);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
                                                              () -> -driverCtrl.getLeftY(), 
                                                              () -> -driverCtrl.getLeftX())
                                                              .withControllerRotationAxis(driverCtrl::getRightX)
                                                              .deadband(SwerveConstants.deadband)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverCtrl::getRightX,
                                                                                              driverCtrl::getRightY).headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivetrain.getSwerveDrive(),
                                                              () -> -driverCtrl.getLeftY(), 
                                                              () -> -driverCtrl.getLeftX())
                                                              .withControllerRotationAxis(() -> driverCtrl.getRawAxis(2))
                                                              .deadband(SwerveConstants.deadband)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

                                                              SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                            .withControllerHeadingAxis(() ->
                                                                                                            Math.sin(
                                                                                                                driverCtrl.getRawAxis(
                                                                                                                    2) *
                                                                                                                Math.PI) *
                                                                                                            (Math.PI *
                                                                                                              2),
                                                                                                        () ->
                                                                                                            Math.cos(
                                                                                                                driverCtrl.getRawAxis(
                                                                                                                    2) *
                                                                                                                Math.PI) *
                                                                                                              (Math.PI *
                                                                                                              2))
                                                                            .headingWhile(true)
                                                                            .translationHeadingOffset(true)
                                                                            .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                0));
  
  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Command driveFieldOrientedDirectAngle = drivetrain.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity = drivetrain.driveFieldOriented(driveRobotOriented);
    // Command driveFieldOrientedDirectAngleKeyboard = drivetrain.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivetrain.driveFieldOriented(driveAngularVelocityKeyboard);

    if(Robot.isSimulation()){
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    }
    else{
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    //Example Intake Roller Bindings
    operatorCtrl.a().whileTrue(intakeRollers.setVoltageCmd(Volts.of(6)));
    operatorCtrl.b().whileTrue(intakeRollers.setVoltageCmd(Volts.of(-6)));
    
    operatorCtrl.x().onTrue(intakeAngle.setPositionCmd(Degrees.of(0)));
    operatorCtrl.y().onTrue(intakeAngle.setPositionCmd(Degrees.of(90)));

    operatorCtrl.povUpRight().whileTrue(indexer.setVoltageCmd(Volts.of(6)));//idk sigmasigmasigmasigma
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
