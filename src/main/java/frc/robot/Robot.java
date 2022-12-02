// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends TimedRobot {

  int loop_counter = 0;
  int prints = 0;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(); 
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
  }

  @Override
  public void robotPeriodic() {
    // System.out.println("swervy");
    if (++loop_counter > 1000) {
      System.out.println("Running " + (prints++));
      loop_counter = 0;
    }

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  


  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // SmartDashboard.putNumber("Counter", counter++);
    //System.out.println(Preferences.getDouble(this.getClass().getName(), 0));
  }
}
