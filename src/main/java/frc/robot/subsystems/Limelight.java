package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.C_Limelight;

public class Limelight implements Subsystem {

    public Limelight() {
        register();
    }

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    private boolean validTarget = false;

        public void UpdateTracking() {
          if (tv.getDouble(-7.0) != 1.0) {
               validTarget = false;
               return;
           }

           validTarget = true;
           
        }

    public double calculateDistance() {

        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double angleToGoal = (C_Limelight.MOUNT_ANGLE + targetOffsetAngle_Vertical) * (Math.PI/180.0);

        double distance = (C_Limelight.GOAL_HEIGHT - C_Limelight.DISTANCE_FROM_FLOOR) / Math.tan(angleToGoal);

        return distance;
        

    }

    public double tx() {

        double offset_from_target = tx.getDouble(-7.0);
        return offset_from_target;
    }

    @Override
    public void periodic() {
        double x = tx.getDouble(-7.0);
        double y = ty.getDouble(-7.0);
        double target = tv.getDouble(-7.0);
        double area = ta.getDouble(-7.0);
        
        Shuffleboard.getTab("graphing").addNumber("LimelightX", ()-> tx.getDouble(-7.0));
        Shuffleboard.getTab("graphing").addNumber("LimelightY", ()-> ty.getDouble(-7.0));
        Shuffleboard.getTab("graphing").addNumber("LimelightArea", ()-> ta.getDouble(-7.0));
        Shuffleboard.getTab("graphing").addNumber("Distance From Target", ()-> calculateDistance());
        

        UpdateTracking();

    }

}
