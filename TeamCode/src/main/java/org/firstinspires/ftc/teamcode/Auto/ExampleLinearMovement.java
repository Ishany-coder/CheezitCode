package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.TeleOp.CoaxialDrive;
import org.firstinspires.ftc.teamcode.TeleOp.CoaxialDrive;

// This is an example file using Squid that takes the robot from its starting position to the point (24,0)
// it continously updates its position and stops when it reaches within 1 inch of the target position
@Autonomous(name="Cheezits Auto SquID", group="Cheezits")
public class ExampleLinearMovement extends LinearOpMode {
    private CoaxialDrive myHardware;
    private DrivetrainSquIDController drivetrain;
    private Pose2d currentPose;
    private Pose2d targetPose;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and drivetrain
        myHardware = new CoaxialDrive(this.hardwareMap);
        drivetrain = new DrivetrainSquIDController();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); // Wait for the start signal

        // Set starting position and target position
        currentPose = new Pose2d(0, 0, new Rotation2d(0)); // Start at (0,0)
        targetPose = new Pose2d(24, 0, new Rotation2d(0)); // Move to (24,0)

        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive()) {
            // Calculate movement using SquIDController
            Pose2d movement = drivetrain.calculate(targetPose, currentPose, new Pose2d(0, 0, new Rotation2d(0)));

            // Convert movement to servo position
            double servoPosition = myHardware.getAngle(movement.getY(), movement.getX());
            myHardware.turn(servoPosition);
            if(movement.getY() > 0){
                myHardware.moveForward();
            }
            else if(movement.getY() < 0){
                myHardware.moveBackward();
            }
            // Update position estimation
            currentPose = new Pose2d(
                    currentPose.getX() + movement.getX() * runtime.seconds(),
                    currentPose.getY() + movement.getY() * runtime.seconds(),
                    new Rotation2d(0)
            );

            // Stop if close to target
            if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.5) {
                break;
            }
            Log.i("MOVING ROBOT ", "TO POSE: " + targetPose);
            Log.i("MOVING ROBOT ", "AT POSE: " + currentPose);
        }
    }
}