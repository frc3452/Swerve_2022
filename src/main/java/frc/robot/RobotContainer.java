// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
// import frc.robot.commands.autonomous.LowBackup;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.WheelDrive;
import frc.robot.util.Util;


public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private final XboxController joystickDrive = new XboxController(0);
        private final XboxController joystickControl = new XboxController(1);

        public boolean fieldRelative = true;
        private boolean constantIntake = false;
        private boolean presetShootetr = false;

        private final Climber climber;
        private final Intake intake;
        private final Shooter shooter;
        public final SwerveDrive swerve;
        private final UpperIndex index;
        private final IntakeActuation actuator;

        private final SendableChooser<Command> chooser = new SendableChooser<Command>();

        private final Red1Auto defaultAuto;

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



                // SmartDashboard <3.
                SmartDashboard.putData("ZeroAzimuth", new ZeroAzimuthCommand(swerve));

                climber.setDefaultCommand(new ClimberCommand(climber, joystickControl));
                swerve.setDefaultCommand(
                                new SwerveDriveCommand(swerve, this::getX, this::getY, this::getOmega,
                                                () -> fieldRelative));
                // new ShooterLogic(intake, index, shooter);

                new JoystickButton(joystickControl, Button.kRightBumper.value)//
                                .whileHeld(new IntakeActuationCommand(actuator, false));

                new JoystickButton(joystickControl, Button.kLeftBumper.value)//
                                .whileHeld(new IntakeActuationCommand(actuator, true));

                new JoystickButton(joystickDrive, Button.kLeftBumper.value)
                                .whileHeld(new IntakeCommand(intake, true));

                new JoystickButton(joystickControl, Button.kY.value)
                                .whileHeld(new IntakeCommand(intake, false));

                new JoystickButton(joystickControl, Button.kY.value)
                                .whileHeld(new ShooterCommand(shooter, false));

                new JoystickButton(joystickControl, Button.kY.value)
                                .whileHeld(new UpperIndexCommand(index, false));

                new JoystickButton(joystickControl, Button.kB.value)
                                .whileHeld(new ShooterCommand(shooter, true));

                new JoystickButton(joystickDrive, Button.kRightBumper.value)
                                .whileHeld(new UpperIndexCommand(index, true));

                new JoystickButton(joystickDrive, Button.kA.value)
                                .whenPressed(new AutoTurn(swerve, 0));

                new JoystickButton(joystickDrive, Button.kB.value)
                                .whenPressed(new AutoTurn(swerve, 2));

                new JoystickButton(joystickDrive, Button.kX.value)
                                .whenPressed(new AutoTurn(swerve, 3));
                
                new JoystickButton(joystickDrive, Button.kY.value)
                                .whenPressed(new AutoTurn(swerve, 1));

                new JoystickButton(joystickDrive, Button.kStart.value)
                                .whenPressed(new InstantCommand(() -> fieldRelative = !fieldRelative));

                // new JoystickButton(joystickDrive, Button.kY.value)
                //                 .whenPressed(new RotateToDegree(swerve, new Rotation2d(0)));


                // new JoystickButton(joystickDrive, Button.kLeftBumper.value)
                // .whenPressed(new InstantCommand(() -> constantIntake = !constantIntake));

                // new JoystickButton(joystickControl, Button.kX.value)
                // .whenPressed(new InstantCommand(() -> presetShootetr = !presetShootetr));

                defaultAuto = new Red1Auto(swerve, intake, index, shooter);

                chooser.setDefaultOption("Shoot", defaultAuto);

                chooser.addOption("test", new TestAuto(swerve, intake, index, shooter));
                chooser.addOption("test2", new TestAuto2(swerve, intake, index, shooter));
                chooser.addOption("Shoot 2 Shoot", new Red2Auto(swerve, intake, index, shooter));
                chooser.addOption("Low Shoot", new Red3Auto(swerve, intake, index, shooter));
                chooser.addOption("Time", new DriveTime(swerve, new ChassisSpeeds(0.0, 5, 0), 2));

                // chooser.addOption("Low Shoot Backup", new LowBackup(swerve, intake, index, shooter));
                // chooser.addOption("Back Up", new Backup(swerve));
                // chooser.addOption("ShootandBackUp", new ExampleAuto(swerve, index, shooter));

                SmartDashboard.putData(chooser);
                
                Shuffleboard.getTab("swervy").addNumber("X", this::getOmega);
                //Shuffleboard.getTab("graphing").addNumber("Motor Controller 1", )

                // new JoystickButton(joystick, Button.kB.value)
                // .whileActiveOnce(new ZeroAzimuthCommand(swerve));
        }

        private double getX() {
                return Util.deadband(joystickDrive.getRawAxis(1) * (1 / 0.25));
        }

        private double getY() {
                return Util.deadband(joystickDrive.getRawAxis(0) * (1 / 0.25));
        }

        private double getOmega() {
                return Util.deadband(joystickDrive.getRawAxis(4)) * Units.degreesToRadians(360);
        }

        private boolean getDrive(Button button) {
                return new JoystickButton(joystickDrive, button.value).get();

        }

        // public void newSwerve(){
        // new SwerveDriveCommand(swerve, this::getX, this::getY, this::getOmega,
        //         () -> fieldRelative);
        // }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
               // return new TestAuto(swerve, intake, index, shooter);
               // return new Red1Auto(swerve, intake, index, shooter);
               Command selected = chooser.getSelected();
               if (selected == null) {
                        return defaultAuto;
                } else {
                        return selected;
                }
        }
}
