package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier limelightX;
    private DoubleSupplier limelightY;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier limelightSupplier;
    private DoubleSupplier limelightVal;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier limelightSupplier, DoubleSupplier limelightX,
    DoubleSupplier limelightY, DoubleSupplier limelightVal) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        this.limelightSupplier = limelightSupplier;

        this.limelightX = limelightX;
        this.limelightY = limelightY;

        this.limelightVal = limelightVal;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);



        /* Drive */
        if (limelightSupplier.getAsBoolean() && limelightVal.getAsDouble() == 1.0) {
            
            // s_Swerve.control(
            // new Translation2d(Limelight.getTargetPose()[2], -Limelight.getTargetPose()[0]), 
            // rotationVal, 
            // false, 
            // true);
            s_Swerve.control(
            new Translation2d(limelightX.getAsDouble(), -limelightY.getAsDouble()), 
            0, 
            false, 
            true);
            
        } else {
            s_Swerve.control(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
        }
        
    }
}