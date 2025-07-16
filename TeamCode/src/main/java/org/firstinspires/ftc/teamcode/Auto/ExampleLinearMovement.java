package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;

// This is an example file using Squid that takes the robot from its starting position to the point (24,0)
// it continously updates its position and stops when it reaches within 1 inch of the target position
@Autonomous(name="Cheezits Auto SquID", group="Cheezits")
public class ExampleLinearMovement extends LinearOpMode {
    private CoaxialDrive myHardware;
    private Pose2d currentPose;
    private Pose2d targetPose;
    double vel = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and drivetrain
        myHardware = new CoaxialDrive(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); // Wait for the start signal

        // Set starting position and target position
        currentPose = new Pose2d(0, 0, new Rotation2d(0)); // Start at (0,0)
        targetPose = new Pose2d(24, 0, new Rotation2d(0)); // Move to (24,0)
        while (opModeIsActive()) {
            // Calculate movement using SquIDController
            myHardware.MoveRobotLinear(targetPose, currentPose, 1);
        }
    }
}