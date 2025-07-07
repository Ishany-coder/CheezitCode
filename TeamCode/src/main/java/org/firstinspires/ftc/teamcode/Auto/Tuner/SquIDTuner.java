package org.firstinspires.ftc.teamcode.Auto.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.Controller.squid.SquIDController;
import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;
@Config
@TeleOp(name="cheezitsSwerve", group="CheezitsTeleOp")
public class SquIDTuner extends LinearOpMode {
    private ElapsedTime loopTime;  // Timer to track loop iteration duration
    private SquIDController squid; // Custom PID controller for movement adjustments
    public static double looptimeAdjuster = 15;

    private CoaxialDrive drive;
    private Pose2d currentPose, targetPose;
    private ElapsedTime timer;

    @Override
    public void runOpMode() {
        drive = new CoaxialDrive(this.hardwareMap);
        timer = new ElapsedTime();

        currentPose = new Pose2d(0, 0, new Rotation2d(0));
        targetPose = new Pose2d(24, 0, new Rotation2d(0));

        waitForStart();

        boolean movingForward = true;

        while (opModeIsActive()) {
            Pose2d movement = calculate(targetPose, currentPose, new Pose2d(0, 0, new Rotation2d(0)));

            drive.moveForward();

            if (Math.abs(currentPose.getX() - targetPose.getX()) < 1) {
                movingForward = !movingForward;
                targetPose = new Pose2d(movingForward ? 24 : 0, 0, new Rotation2d(0));
                timer.reset();
            }

            currentPose = new Pose2d(
                    currentPose.getX() + movement.getX(),
                    currentPose.getY() + movement.getY(),
                    new Rotation2d(movement.getHeading())
            );

            // Display tuning parameters
            telemetry.addData("Loop Time Adjuster", looptimeAdjuster);
            telemetry.addData("Current X", currentPose.getX());
            telemetry.addData("Target X", targetPose.getX());
            telemetry.addData("X error: ", currentPose.getX() - targetPose.getX());
            telemetry.addData("Y error: ", currentPose.getY() - targetPose.getY());
            telemetry.addData("Heading error: ", currentPose.getHeading() - targetPose.getHeading());
            telemetry.update();
        }
    }
    public Pose2d calculate(Pose2d targetPose, Pose2d currentPose, Pose2d currentVelocity) {
        // Compute the magnitude (length) of the velocity vector
        double currentVelMag = currentVelocity.getTranslation().getNorm();

        // Calculate the time elapsed in the control loop
        double deltaSeconds = loopTime.seconds();

        // Adjust velocity magnitude based on loop time
        currentVelMag *= deltaSeconds;

        // Determine the direction of movement using atan2
        double velAngle = Math.atan2(currentVelocity.getY(), currentVelocity.getX());

        // Estimate how far the robot will travel due to velocity
        double distance = Math.max(getDistanceFromVelocity(currentVelMag), 0);

        // Adjust current position by adding the predicted travel distance in the velocity direction
        currentPose = currentPose.plus(new Transform2d(
                new Translation2d(Math.cos(velAngle) * distance, Math.sin(velAngle) * distance),
                new Rotation2d()
        ));

        // Calculate the remaining distance to the target
        double magnitude = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        // Use the PID controller to generate an adjustment based on the error
        magnitude = squid.calculate(magnitude, 0);

        // Compute the direction to the target
        Translation2d delta = targetPose.getTranslation().minus(currentPose.getTranslation());
        double posAngle = Math.atan2(delta.getY(), delta.getX());

        // Reset the loop timer for the next iteration
        loopTime.reset();

        // Return a new Pose2d with the computed movement correction
        return new Pose2d(
                Math.cos(posAngle) * magnitude,  // X component
                Math.sin(posAngle) * magnitude,  // Y component
                new Rotation2d()                 // Rotation remains unchanged
        );
    }

    /**
     * Estimates how far the robot will travel based on its velocity.
     * Uses a quadratic regression model to predict distance traveled per control loop.
     *
     * @param velocity The current velocity magnitude
     * @return The estimated travel distance
     */
    private static double getDistanceFromVelocity(double velocity) {
        velocity *= looptimeAdjuster;  // Adjust velocity based on tuning constant
        // Quadratic regression equation to approximate travel distance
        return 0.00286 * velocity * velocity + 0.304 * velocity - 0.837;
    }
}