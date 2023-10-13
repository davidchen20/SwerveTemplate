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
  
  private NetworkTableEntry tv = null;
  private NetworkTableEntry tx = null;
  private NetworkTableEntry ty = null;
  private NetworkTableEntry ta = null;
  private NetworkTableEntry botpose = null;
  private NetworkTableEntry targetpose = null;
  private NetworkTableEntry tl = null;
  private NetworkTableEntry cl = null;

  private Alliance alliance = null;

  private boolean initializedVals = false;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public Limelight() {
    this.alliance = DriverStation.getAlliance();
    try {
      initializedVals = true;
      this.tv = table.getEntry("tv");
      this.tx = table.getEntry("tx");
      this.ty = table.getEntry("ty");
      this.ta = table.getEntry("ta");

      if (alliance.equals(Alliance.Blue)) {
        this.botpose = table.getEntry("botpose_wpiblue");
      } else {
        this.botpose = table.getEntry("botpose_wpired");
      }

      this.targetpose = table.getEntry("targetpose_robotspace");
      this.tl = table.getEntry("tl");
      this.cl = table.getEntry("cl");

    } catch (Exception e) {}
  }

  public double getValue() {
    return tv.getDouble(0.0);
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getA() {
    return ta.getDouble(0.0);
  }

  public double[] getBotPose() {
    return botpose.getDoubleArray(new double[7]);
  }

  public double[] getTargetPose() {
    return targetpose.getDoubleArray(new double[7]);
  }

  public double getTargetLatency() {
    return tl.getDouble(0.0);
  }

  public double getCaptureLatency() {
    return cl.getDouble(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limelihgt Vals", initializedVals);

    try {
      initializedVals = true;
      this.tv = table.getEntry("tv");
      this.tx = table.getEntry("tx");
      this.ty = table.getEntry("ty");
      this.ta = table.getEntry("ta");

      if (alliance.equals(Alliance.Blue)) {
        this.botpose = table.getEntry("botpose_wpiblue");
      } else {
        this.botpose = table.getEntry("botpose_wpired");
      }

      this.targetpose = table.getEntry("targetpose_robotspace");
      this.tl = table.getEntry("tl");
      this.cl = table.getEntry("cl");

    } catch (Exception e) {}
  }
}
