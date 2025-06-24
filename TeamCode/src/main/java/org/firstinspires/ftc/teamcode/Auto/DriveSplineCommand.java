package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.TeleOp.CoaxialDrive;

public class DriveSplineCommand extends CommandBase {
    private final CoaxialDrive drive;
    private final DrivetrainSquIDController squidController;
    private Pose2d currentPose;
    private final Pose2d targetPose;
    private final long duration;
    private long startTime;

    /**
     * Constructs a DriveSplineCommand to move the robot along a smooth trajectory
     * using the SquID controller and servo-based drivetrain.
     *
     * @param drive           The Drive subsystem controlling the servos.
     * @param squidController The SquID controller for motion correction.
     * @param start           The initial position (Pose2d) of the robot.
     * @param end             The target position (Pose2d) for the robot to reach.
     * @param timeSeconds     The duration (in seconds) for the movement.
     */
    public DriveSplineCommand(CoaxialDrive drive, DrivetrainSquIDController squidController, Pose2d start, Pose2d end, double timeSeconds) {
        this.drive = drive;  // Assign the Drive subsystem to control movement.
        this.squidController = squidController;  // Assign SquID for trajectory adjustments.

        this.currentPose = start;  // Set the initial pose of the robot.
        this.targetPose = end;  // Set the target pose where the robot should move.

        // Convert time duration from seconds to milliseconds for precise timing.
        this.duration = (long) (timeSeconds * 1000);

        // Register this command as requiring control of the Drive subsystem.
        addRequirements((Subsystem) drive);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Estimate movement correction using SquIDController
        Pose2d movement = squidController.calculate(targetPose, currentPose, new Pose2d(0, 0, new Rotation2d(0)));

        // Convert movement into an angle for servos
        double angle = Math.toDegrees(Math.atan2(movement.getY(), movement.getX()));

        // Schedule commands sequentially: first turn, then move forward
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> drive.turn(angle)), // Turn wheels first
                        new InstantCommand(() -> drive.moveForward()) // Then move forward
                )
        );

        // Update currentPose with corrected rotation type
        currentPose = new Pose2d(
                currentPose.getX() + movement.getX(),
                currentPose.getY() + movement.getY(),
                new Rotation2d(movement.getHeading()) // Convert heading correction
        );
    }

    @Override
    public boolean isFinished() {
        //If the robot reaches its end destination stop the code
        return (currentPose.getX() - targetPose.getX() >= 1) &&
                (currentPose.getY() - targetPose.getY() > 1) &&
                (currentPose.getHeading() - targetPose.getHeading() > 1);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}