// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final static XboxController joystickDrive = new XboxController(0);
  private final static XboxController joystickControl = new XboxController(1);
  private final static XboxController joystick = new XboxController(2);


  private final Climber climber;
  private final Intake intake;
  private final Shooter shooter;
  private final SwerveDrive swerve;



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    WheelDrive backRight = new WheelDrive(C_SwerveModules.BACK_RIGHT_AZIMUTH, C_SwerveModules.BACK_RIGHT_SPEED);
    WheelDrive backLeft = new WheelDrive(C_SwerveModules.BACK_LEFT_AZIMUTH, C_SwerveModules.BACK_LEFT_SPEED);
    WheelDrive frontRight = new WheelDrive(C_SwerveModules.FRONT_RIGHT_AZIMUTH, C_SwerveModules.FRONT_RIGHT_SPEED);
    WheelDrive frontLeft = new WheelDrive(C_SwerveModules.FRONT_LEFT_AZIMUTH, C_SwerveModules.FRONT_LEFT_SPEED);

    climber = new Climber(C_Climber.LEFT_CLIMBER,C_Climber.RIGHT_CLIMBER);
    intake = new Intake(C_Intake.INTAKE, C_Index.LOWER_INDEX);
    shooter = new Shooter(C_Shooter.LEFT_SHOOTER, C_Shooter.RIGHT_SHOOTER, C_Index.UPPER_INDEX);
    swerve = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);


    climber.setDefaultCommand(new ClimberCommand(climber, joystickControl));
    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, joystickDrive));

    new JoystickButton(joystickControl, Button.kA.value)
    .whileHeld(new IntakeCommand(intake));

    new JoystickButton(joystickControl, Button.kB.value)
      .whileHeld(new ShooterCommand(shooter, joystickControl));
  
    new JoystickButton(joystick, Button.kB.value)
      .whileActiveOnce(new ZeroAzimuthCommand(swerve));
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
    // An ExampleCommand will run in autonomous
    return null;
  }
}
