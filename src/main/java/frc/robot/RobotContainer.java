// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.Autonomous.DriveDistance;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  private final Climber climber;
  private final Intake intake;
  private final Shooter shooter;
  public final SwerveDrive swerve;
  private final UpperIndex index;


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
    shooter = new Shooter(C_Shooter.LEFT_SHOOTER, C_Shooter.RIGHT_SHOOTER);
    swerve = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);
    index = new UpperIndex(C_Index.UPPER_INDEX);

    SmartDashboard.putData("ZeroAzimuth", new ZeroAzimuthCommand(swerve));

    climber.setDefaultCommand(new ClimberCommand(climber, joystickControl));
    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, joystickDrive));

    new JoystickButton(joystickControl, Button.kA.value)
    .whileHeld(new IntakeCommand(intake));

    new JoystickButton(joystickControl, Button.kB.value)
      .whileHeld(new ShooterCommand(shooter));

    new JoystickButton(joystickControl, Button.kY.value)
      .whileHeld(new UpperIndexCommand(index));
  
    // new JoystickButton(joystick, Button.kB.value)
    //   .whileActiveOnce(new ZeroAzimuthCommand(swerve));
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
    return new DriveDistance(swerve);
  }
}
