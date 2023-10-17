// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  
  private static NetworkTableEntry tv = null;
  private static NetworkTableEntry tx = null;
  private static NetworkTableEntry ty = null;
  private static NetworkTableEntry ta = null;
  private static NetworkTableEntry botpose = null;
  private static NetworkTableEntry targetpose = null;
  private static NetworkTableEntry tl = null;
  private static NetworkTableEntry cl = null;

  private Alliance alliance = null;

  private boolean initializedVals = false;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public Limelight() {
    this.alliance = DriverStation.getAlliance();
    try {
      initializedVals = true;
      Limelight.tv = table.getEntry("tv");
      Limelight.tx = table.getEntry("tx");
      Limelight.ty = table.getEntry("ty");
      Limelight.ta = table.getEntry("ta");

      if (alliance.equals(Alliance.Blue)) {
        Limelight.botpose = table.getEntry("botpose_wpiblue");
      } else {
        Limelight.botpose = table.getEntry("botpose_wpired");
      }

      Limelight.targetpose = table.getEntry("targetpose_robotspace");
      Limelight.tl = table.getEntry("tl");
      Limelight.cl = table.getEntry("cl");

      System.out.println(Limelight.tx.getDouble(0));

    } catch (Exception e) {}
  }

  public static double getValue() {
    return tv.getDouble(0.0);
  }

  public static double getX() {
    return tx.getDouble(0.0);
  }

  public static double getY() {
    return ty.getDouble(0.0);
  }

  public static double getArea() {
    return ta.getDouble(0.0);
  }

  public static double[] getBotPose() {
    return botpose.getDoubleArray(new double[6]);
  }

  public static double[] getTargetPose() {
    return targetpose.getDoubleArray(new double[6]);
  }

  public static double getTargetLatency() {
    return tl.getDouble(0.0);
  }

  public static double getCaptureLatency() {
    return cl.getDouble(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limelihgt Vals", initializedVals);

    try {
      initializedVals = true;
      Limelight.tv = table.getEntry("tv");
      Limelight.tx = table.getEntry("tx");
      Limelight.ty = table.getEntry("ty");
      Limelight.ta = table.getEntry("ta");

      if (alliance.equals(Alliance.Blue)) {
        Limelight.botpose = table.getEntry("botpose_wpiblue");
      } else {
        Limelight.botpose = table.getEntry("botpose_wpired");
      }

      Limelight.targetpose = table.getEntry("targetpose_robotspace");
      Limelight.tl = table.getEntry("tl");
      Limelight.cl = table.getEntry("cl");

    } catch (Exception e) {}
  }
}
