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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterFront;
  private CANSparkMax shooterBack;
  private final SparkMaxPIDController pidControllerShooterFront;
  private final SparkMaxPIDController pidControllerShooterBack;
  double frontSpeed;
  double backSpeed;
  private RelativeEncoder EncoderShooterFront;
  private RelativeEncoder EncoderShooterBack;
  private Limelight limelight;
  private double distance;


  /** Creates a new Shooter. */
  public Shooter(int shooterFront, int shooterBack) {
    this.shooterFront = new CANSparkMax(shooterFront, MotorType.kBrushless);
    this.shooterFront.setSmartCurrentLimit(40);
    this.shooterFront.setInverted(false);
    this.shooterFront.setIdleMode(IdleMode.kCoast);

    this.pidControllerShooterFront = this.shooterFront.getPIDController();
    this.pidControllerShooterFront.setP(1e-4);
    this.pidControllerShooterFront.setFF(0.000195);
  
    this.EncoderShooterFront = this.shooterFront.getEncoder();

    this.shooterBack = new CANSparkMax(shooterBack, MotorType.kBrushless);
    this.shooterBack.setSmartCurrentLimit(40);
    this.shooterBack.setInverted(false);
    this.shooterBack.setIdleMode(IdleMode.kCoast);

    this.pidControllerShooterBack = this.shooterBack.getPIDController();
    this.pidControllerShooterBack.setP(1e-4);
    this.pidControllerShooterBack.setFF(0.000195);

    this.EncoderShooterBack = this.shooterBack.getEncoder();
    
    Shuffleboard.getTab("graphing").addNumber("front rpm",()-> this.EncoderShooterFront.getVelocity());
    Shuffleboard.getTab("graphing").addNumber("back rpm",()-> this.EncoderShooterBack.getVelocity());
  }

  public void shoot() {
    shoot(false);
  }

  public void shoot(boolean inverted) {
    // distance = limelight.calculateDistance();
    // System.out.println(distance);
    // System.out.println("I was here");
    if (!inverted){ 
      frontSpeed = Preferences.getDouble("frontSpeed", 2337.0);
      backSpeed = Preferences.getDouble("backSpeed", 2714.0);
    } else {
      frontSpeed = Preferences.getDouble("frontSpeed", 2337.0)*-1;
      backSpeed = Preferences.getDouble("backSpeed", 2714.0)*-1;
    }
    this.pidControllerShooterFront.setReference(frontSpeed,CANSparkMax.ControlType.kVelocity);
    this.pidControllerShooterBack.setReference(backSpeed,CANSparkMax.ControlType.kVelocity);
  }

  public void lowShoot() {
    lowshoot(false);
  }
  public void lowshoot(boolean inverted) {
    frontSpeed = Preferences.getDouble("frontSpeed", 2337);
    this.pidControllerShooterFront.setReference(frontSpeed,CANSparkMax.ControlType.kVelocity);
  }



  public void stop() {
    this.shooterFront.set(0);
    this.shooterBack.set(0);
  }
}
