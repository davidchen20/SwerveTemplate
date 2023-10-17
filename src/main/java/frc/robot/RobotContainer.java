package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.wrappers.Controller;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Controller driver = new Controller(0, Constants.stickDeadband);

    /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Limelight limelight = new Limelight();
    private final Swerve s_Swerve = new Swerve();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getLeftJoyY(), 
                () -> -driver.getLeftJoyX(), 
                () -> -driver.getRightJoyX(), 
                () -> driver.getLBButton().getAsBoolean(),
                () -> driver.getRBButton().getAsBoolean(),
                () -> Limelight.getTargetPose()[2],
                () -> Limelight.getTargetPose()[0],
                () -> Limelight.getValue()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driver.getSTARTButton().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        driver.getAButton().onTrue(new AprilTagHerder());
        // driver.getAButton().onFalse(new InstantCommand(s_Swerve::stop, s_Swerve));
        // driver.getAButton().onTrue(new InstantCommand(() -> s_Swerve.control(new Translation2d(Limelight.getTargetPose()[1], Limelight.getTargetPose()[0]), 0, false, false)));
        // driver.getAButton().whileTrue(new InstantCommand(() -> s_Swerve.control(new Translation2d(100, 10), 1, false, true)));
        // driver.getAButton().onTrue(new InstantCommand(() -> s_Swerve.followTrajectoryCommand(s_Swerve.generateTagTrajecotry(), false), s_Swerve));
        // driver.getAButton().whileTrue(new InstantCommand() ->  s_Swerve.control(
        //     new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
        //     rotationVal * Constants.Swerve.maxAngularVelocity, 
        //     !robotCentricSup.getAsBoolean(), 
        //     true
        // ););
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new TwoTwoSix(s_Swerve);
    }
}
