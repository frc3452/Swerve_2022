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
import frc.robot.commands.autonomous.BackandShoot;
import frc.robot.commands.autonomous.Backup;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.WheelDrive;
import frc.robot.util.Util;

public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private final XboxController joystickDrive = new XboxController(0);
        private final XboxController joystickControl = new XboxController(1);

        private boolean fieldRelative = false;

        private final Climber climber;
        private final Intake intake;
        private final Shooter shooter;
        public final SwerveDrive swerve;
        private final UpperIndex index;
        private final IntakeActuation actuator;

        private final SendableChooser<Command> chooser = new SendableChooser<Command>();

        private final BackandShoot defaultAuto;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                climber = new Climber(C_Climber.LEFT_CLIMBER, C_Climber.RIGHT_CLIMBER);
                intake = new Intake(C_Intake.INTAKE, C_Index.LOWER_INDEX);
                shooter = new Shooter(C_Shooter.LEFT_SHOOTER, C_Shooter.RIGHT_SHOOTER);
                swerve = new SwerveDrive();
                index = new UpperIndex(C_Index.UPPER_INDEX);
                actuator = new IntakeActuation(C_Actuator.ACTUATOR);

                // SmartDashboard.
                SmartDashboard.putData("ZeroAzimuth", new ZeroAzimuthCommand(swerve));

                climber.setDefaultCommand(new ClimberCommand(climber, joystickControl));
                swerve.setDefaultCommand(
                                new SwerveDriveCommand(swerve, this::getX, this::getY, this::getOmega, () -> fieldRelative));

                new JoystickButton(joystickControl, Button.kA.value)
                                .whileHeld(new IntakeCommand(intake, false));

                new JoystickButton(joystickDrive, Button.kA.value)
                                .whileHeld(new IntakeCommand(intake, true));

                new JoystickButton(joystickControl, Button.kRightBumper.value)
                                .whileHeld(new IntakeActuationCommand(actuator, false));

                new JoystickButton(joystickControl, Button.kLeftBumper.value)
                                .whileHeld(new IntakeActuationCommand(actuator, true));

                new JoystickButton(joystickControl, Button.kB.value)
                                .whileHeld(new ShooterCommand(shooter, true));

                new JoystickButton(joystickControl, Button.kY.value)
                                .whileHeld(new UpperIndexCommand(index, true));

                new JoystickButton(joystickDrive, Button.kY.value)
                                .whileHeld(new UpperIndexCommand(index, false));

                new JoystickButton(joystickDrive, Button.kStart.value)
                                .whenPressed(new InstantCommand(() -> fieldRelative = !fieldRelative));

                defaultAuto = new BackandShoot(swerve, index, shooter);

                chooser.setDefaultOption("Default Auto", defaultAuto);

                chooser.addOption("BackandShoot", new BackandShoot(swerve, index, shooter));
                chooser.addOption("Back Up", new Backup(swerve));
                // chooser.addOption("ShootandBackUp", new ExampleAuto(swerve, index, shooter));

                SmartDashboard.putData(chooser);

                Shuffleboard.getTab("swervy").addNumber("X", this::getOmega);

                // new JoystickButton(joystick, Button.kB.value)
                // .whileActiveOnce(new ZeroAzimuthCommand(swerve));
        }

        private double getX() {
                return Util.deadband(-joystickDrive.getRawAxis(1));
        }

        private double getY() {
                return Util.deadband(-joystickDrive.getRawAxis(0));
        }

        private double getOmega() {
                return Util.deadband(joystickDrive.getRawAxis(2)-joystickDrive.getRawAxis(3));
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
