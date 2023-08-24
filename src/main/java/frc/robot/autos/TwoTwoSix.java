// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoTwoSix extends SequentialCommandGroup {
  /** Creates a new TwoTwoSix. */
  public TwoTwoSix(Swerve s_Swerve) {
  
        
    var rotationController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);

    PathPlannerTrajectory two = PathPlanner.loadPath("2", new PathConstraints(4, 3));
    PathPlannerTrajectory six = PathPlanner.loadPath("6", new PathConstraints(4, 3));

    PPSwerveControllerCommand ppp = new PPSwerveControllerCommand(
                two, 
                s_Swerve::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                rotationController,
                s_Swerve::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                s_Swerve // Requires this drive subsystem
             );
        
    // Add your commands in the addCommands() call, e.g.
    addCommands(new InstantCommand(() -> s_Swerve.resetOdometry(two.getInitialPose())), ppp);
  }
}
