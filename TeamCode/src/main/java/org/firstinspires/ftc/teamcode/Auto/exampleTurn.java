package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CheezitsTeleop.Drive;

@Autonomous(name="Cheezits Turn Test", group="Cheezits")
public class exampleTurn extends LinearOpMode {

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

        // Set start pose (at 0 degrees) and target pose (90-degree rotation)
        currentPose = new Pose2d(0, 0, new Rotation2d(0)); // Facing forward
        targetPose = new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))); // Rotate to 90 degrees

        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive()) {
            // Calculate turning adjustment using SquIDController
            Pose2d turnAdjustment = drivetrain.calculate(targetPose, currentPose, new Pose2d(0, 0, new Rotation2d(0)));

            // Extract rotation adjustment
            double turnAngle = turnAdjustment.getRotation().getRadians();

            // Convert the turn adjustment to a servo position
            double servoPosition = (turnAngle / Math.PI) + 0.5; // Normalize to [0,1]
            myHardware.turnDriveMotors(servoPosition);

            // Update estimated rotation
            currentPose = new Pose2d(
                    0, 0, // No translation, just rotation
                    new Rotation2d(currentPose.getRotation().getRadians() + (turnAngle * runtime.seconds()))
            );

            // Stop when close to target (within 2 degrees)
            if (Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()) < Math.toRadians(2)) {
                break;
            }

            telemetry.addData("Target Rotation", Math.toDegrees(targetPose.getRotation().getRadians()));
            telemetry.addData("Current Rotation", Math.toDegrees(currentPose.getRotation().getRadians()));
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }

        myHardware.stopServos(); // Stop servos after turning
    }
}