package frc.lib.wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public static double getHorizontalOffset() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public static double getVerticalOffset() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public static double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public static double getValue() {
        return limelight.getEntry("tv").getDouble(0.0);
    }

    public static double getShortSide() {
        return limelight.getEntry("tshort").getDouble(0.0);
    }

    public static double getLongSide() {
        return limelight.getEntry("tlong").getDouble(0.0);
    }

    public static double getHorizontalSide() {
        return limelight.getEntry("thor").getDouble(0.0);
    }

    public static double getVerticalSide() {
        return limelight.getEntry("tvert").getDouble(0.0);
    }
}
