// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTagHerder extends SequentialCommandGroup {
  /** Creates a new AprilTagHerder. */
  // Swerve s_Swerve = new Swerve();

  
  
  
  public AprilTagHerder() {
    
    double x = Limelight.getTargetPose()[2];
    double y = Limelight.getTargetPose()[0];
    
    
    if(Math.abs(x) > 0.09 && Math.abs(y) > 0.1) {
      PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(1, 0.5), 
      new PathPoint(Swerve.getInstance().getPose().getTranslation(), Swerve.getInstance().getPose().getRotation()),
      new PathPoint(new Translation2d(x, y), new Rotation2d(0)));
      PPSwerveControllerCommand ppp = new PPSwerveControllerCommand(
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
      
      // Not close enough, generate trajectory

      // Add follow command
      addCommands(ppp);
        
    } else {
      
      // is close enough

      // turn leds green or smth 
      addCommands();

    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // if this doesnt work do like teleop swerrve

   
    

    
  }
}
