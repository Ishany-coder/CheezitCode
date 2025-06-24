package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CheezitsTeleop.Drive;

// This is an example file using Squid that takes the robot from its starting position to the point (24,0)
// it continously updates its position and stops when it reaches within 1 inch of the target position
@Autonomous(name="Cheezits Auto SquID", group="Cheezits")
public class ExampleLinearMovement extends LinearOpMode {
    private Drive myHardware;
    private DrivetrainSquIDController drivetrain;
    private Pose2d currentPose;
    private Pose2d targetPose;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and drivetrain
        myHardware = new Drive(this.hardwareMap);
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
            myHardware.turnDriveMotors(servoPosition);

            // Update position estimation
            currentPose = new Pose2d(
                    currentPose.getX() + movement.getX() * runtime.seconds(),
                    currentPose.getY() + movement.getY() * runtime.seconds(),
                    new Rotation2d(0)
            );

            // Stop if close to target
            if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 1) {
                break;
            }

            telemetry.addData("Target Pose", targetPose);
            telemetry.addData("Current Pose", currentPose);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
        myHardware.stopServos();
    }
}