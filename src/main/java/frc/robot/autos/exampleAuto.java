package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class exampleAuto extends SequentialCommandGroup {
    
    public exampleAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(0.25,0)),  //new Translation2d(1, 1), new Translation2d(2, -1)
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(3, 0, new Rotation2d(Math.PI)),
        //         config);

        Trajectory t1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, Rotation2d.fromDegrees(0)), new ArrayList<Translation2d>(), new Pose2d(1.5, 0, Rotation2d.fromDegrees(90)), config);

        Trajectory t2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.5,0, Rotation2d.fromDegrees(90)), new ArrayList<Translation2d>(), new Pose2d(3, 0, Rotation2d.fromDegrees(180)), config);
        
        Trajectory tf = t1.concatenate(t2);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var rotationController =
            new PIDController(
                Constants.AutoConstants.kPThetaController, 0, 0);
        

        SwerveControllerCommand swerveControllerCommand_one =
            new SwerveControllerCommand(
                t1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        PathPlannerTrajectory testPath = PathPlanner.loadPath("testPath", new PathConstraints(4, 3));
        // SwerveControllerCommand ppp =
        //     new SwerveControllerCommand(
        //         testPath,
        //         s_Swerve::getPose,
        //         Constants.Swerve.swerveKinematics,
        //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //         thetaController,
        //         s_Swerve::setModuleStates,
        //         s_Swerve);

        PPSwerveControllerCommand ppp = new PPSwerveControllerCommand(
                testPath, 
                s_Swerve::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                rotationController,
                s_Swerve::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                s_Swerve // Requires this drive subsystem
             );
        

        PPSwerveControllerCommand test = s_Swerve.generateCommand(testPath);
                    
        // Command path = Swerve.getInstance().followTrajectoryCommand(testPath, true);
        // addCommands(
        //     new InstantCommand(() -> s_Swerve.resetOdometry(tf.getInitialPose())),
        //     swerveControllerCommand_one,
        //     new WaitCommand(1),
        //     swerveControllerCommand_two
        // );
        addCommands(new InstantCommand(() -> s_Swerve.resetOdometry(testPath.getInitialPose())), ppp);
    }
}