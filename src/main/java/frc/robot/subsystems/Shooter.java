// CopyBack (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterFront;
  private CANSparkMax shooterBack;
  private final SparkMaxPIDController pidControllerFront;
  private final SparkMaxPIDController pidControllerBack;
  double frontSpeed;
  double backSpeed;
  private RelativeEncoder frontEncoder;
  private RelativeEncoder backEncoder;


  /** Creates a new Shooter. */
  public Shooter(int shooterFront, int shooterBack) {
    this.shooterFront = new CANSparkMax(shooterFront, MotorType.kBrushless);
    this.shooterFront.setSmartCurrentLimit(40);
    this.shooterFront.setInverted(false);
    this.shooterFront.setIdleMode(IdleMode.kCoast);

    this.pidControllerFront = this.shooterFront.getPIDController();
    this.pidControllerFront.setP(1e-4);
    this.pidControllerFront.setFF(0.000195);
  
    this.frontEncoder = this.shooterFront.getEncoder();

    this.shooterBack = new CANSparkMax(shooterBack, MotorType.kBrushless);
    this.shooterBack.setSmartCurrentLimit(40);
    this.shooterBack.setInverted(false);
    this.shooterBack.setIdleMode(IdleMode.kCoast);

    this.pidControllerBack = this.shooterBack.getPIDController();
    this.pidControllerBack.setP(1e-4);
    this.pidControllerBack.setFF(0.000175);

    this.backEncoder = this.shooterBack.getEncoder();
    
    Preferences.initDouble("frontSpeed", 0.0);
    Preferences.initDouble("backSpeed", 0.0);

    Shuffleboard.getTab("graphing").addNumber("front rpm",()-> this.frontEncoder.getVelocity());
    Shuffleboard.getTab("graphing").addNumber("back rpm",()-> this.backEncoder.getVelocity());
  }

  public void shoot() {
    frontSpeed = Preferences.getDouble("frontSpeed", .85);
    this.pidControllerFront.setReference(frontSpeed,CANSparkMax.ControlType.kVelocity);
    backSpeed = Preferences.getDouble("backSpeed", .85);
    this.pidControllerBack.setReference(backSpeed,CANSparkMax.ControlType.kVelocity);
  }

  public void stop() {
    this.shooterFront.set(0);
    this.shooterBack.set(0);
  }
}
