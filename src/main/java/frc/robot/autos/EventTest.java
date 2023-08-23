// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EventTest extends SequentialCommandGroup {
  /** Creates a new EventTest. */
  public EventTest(Swerve s_Swerve){
    var rotationController =
        new PIDController(
            Constants.AutoConstants.kPThetaController, 0, 0);

    PathPlannerTrajectory testPath = PathPlanner.loadPath("eventTest", new PathConstraints(4, 3));

    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("intakeDown", new InstantCommand(() -> Robot.m_robotContainer.intake.extendIntake()));
    eventMap.put("runIn", new InstantCommand(() -> Robot.m_robotContainer.intake.runIn()));
    eventMap.put("retract", new InstantCommand(() -> Robot.m_robotContainer.intake.retractIntake()));
    eventMap.put("stop", new InstantCommand(() -> Robot.m_robotContainer.intake.stop()));

    FollowPathWithEvents follower = new FollowPathWithEvents(new PPSwerveControllerCommand(
      testPath, 
      s_Swerve::getPose, // Pose supplier
      Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
      new PIDController(Constants.AutoConstants.kPXController, 0, 0),
      new PIDController(Constants.AutoConstants.kPYController, 0, 0),
      rotationController,
      s_Swerve::setModuleStates, // Module states consumer
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      s_Swerve // Requires this drive subsystem
   ), testPath.getMarkers(), eventMap);
    
    
                
    // Command path = Swerve.getInstance().followTrajectoryCommand(testPath, true);
    // addCommands(
    //     new InstantCommand(() -> s_Swerve.resetOdometry(tf.getInitialPose())),
    //     swerveControllerCommand_one,
    //     new WaitCommand(1),
    //     swerveControllerCommand_two
    // );
    addCommands(new InstantCommand(() -> s_Swerve.resetOdometry(testPath.getInitialPose())), follower);
  }
}
