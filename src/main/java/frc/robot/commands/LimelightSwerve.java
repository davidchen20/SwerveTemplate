// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class LimelightSwerve extends CommandBase {
  /** Creates a new LimelightSwerve. */
  public BooleanSupplier supplier;
  public PPSwerveControllerCommand ppp;
  public LimelightSwerve(BooleanSupplier supplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Swerve.getInstance());
    this.supplier = supplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double x = Limelight.getTargetPose()[2];
    double y = Limelight.getTargetPose()[0];
    
    if(Math.abs(x) > 0.09 && Math.abs(y) > 0.1 && Limelight.getValue() == 1.0) {
      PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(1, 0.5), 
      new PathPoint(Swerve.getInstance().getPose().getTranslation(), Swerve.getInstance().getPose().getRotation()),
      new PathPoint(new Translation2d(x, y), new Rotation2d(0)));
      this.ppp = new PPSwerveControllerCommand(
                    traj, 
                    Swerve.getInstance()::getPose, // Pose supplier
                    Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    new PIDController(1, 0, 0),
                    Swerve.getInstance()::setModuleStates, // Module states consumer
                    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                    Swerve.getInstance() // Requires this drive subsystem
                 );
    }             

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ppp.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !supplier.getAsBoolean();
  }
}
