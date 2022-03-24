// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.DriveDistance;
import frc.robot.commands.autonomous.ExampleAuto;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final XboxController joystickDrive = new XboxController(0);
  private final XboxController joystickControl = new XboxController(1);

  private final Climber climber;
  private final Intake intake;
  private final Shooter shooter;
  public final SwerveDrive swerve;
  private final UpperIndex index;
  private final IntakeActuation actuator;

  private final SendableChooser<Command> chooser = new SendableChooser<Command>();
  
  private ExampleAuto defaultAuto;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    WheelDrive backRight = new WheelDrive(C_SwerveModules.BACK_RIGHT_AZIMUTH, C_SwerveModules.BACK_RIGHT_SPEED);
    WheelDrive backLeft = new WheelDrive(C_SwerveModules.BACK_LEFT_AZIMUTH, C_SwerveModules.BACK_LEFT_SPEED);
    WheelDrive frontRight = new WheelDrive(C_SwerveModules.FRONT_RIGHT_AZIMUTH, C_SwerveModules.FRONT_RIGHT_SPEED);
    WheelDrive frontLeft = new WheelDrive(C_SwerveModules.FRONT_LEFT_AZIMUTH, C_SwerveModules.FRONT_LEFT_SPEED);

    climber = new Climber(C_Climber.LEFT_CLIMBER, C_Climber.RIGHT_CLIMBER);
    intake = new Intake(C_Intake.INTAKE, C_Index.LOWER_INDEX);
    shooter = new Shooter(C_Shooter.LEFT_SHOOTER, C_Shooter.RIGHT_SHOOTER);
    swerve = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);
    index = new UpperIndex(C_Index.UPPER_INDEX);
    actuator = new IntakeActuation(C_Actuator.ACTUATOR);

    // SmartDashboard.
    SmartDashboard.putData("ZeroAzimuth", new ZeroAzimuthCommand(swerve));

    climber.setDefaultCommand(new ClimberCommand(climber, joystickControl));
    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, joystickDrive));

    new JoystickButton(joystickControl, Button.kA.value)
        .whileHeld(new IntakeCommand(intake));
  
    new JoystickButton(joystickControl, Button.kRightBumper.value)
        .whileHeld(new IntakeActuationCommand(actuator));

    new JoystickButton(joystickControl, Button.kB.value)
        .whileHeld(new ShooterCommand(shooter, true));

    new JoystickButton(joystickControl, Button.kY.value)
        .whileHeld(new UpperIndexCommand(index));

    new JoystickButton(joystickDrive, Button.kStart.value)
        .whenPressed(new InstantCommand(() -> {
          SwerveDrive.isFieldOriented = !SwerveDrive.isFieldOriented;
          // swerve.zero();
        }));

    defaultAuto = new ExampleAuto(swerve, index, shooter);

    chooser.setDefaultOption("Default Auto", defaultAuto);
 
    chooser.addOption("Other A", new ExampleAuto(swerve, index, shooter));
    chooser.addOption("Other B", new ExampleAuto(swerve, index, shooter));
    chooser.addOption("Other C", new ExampleAuto(swerve, index, shooter));

    // Shuffleboard.getTab("A").add("choosy", chooser);
    SmartDashboard.putData(chooser);

    // new JoystickButton(joystick, Button.kB.value)
    // .whileActiveOnce(new ZeroAzimuthCommand(swerve));
  }

  public static double deadband(double value) {
    return deadband(value, Constants.DEFAULT_DEADBAND);
  }

  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new ExampleAuto(swerve, index, shooter);
    Command selected = chooser.getSelected();
    if (selected == null) {
      return defaultAuto;
    } else {
      return selected;
    }
  }
}
