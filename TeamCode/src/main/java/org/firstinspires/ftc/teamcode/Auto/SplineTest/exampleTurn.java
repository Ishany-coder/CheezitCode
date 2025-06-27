package org.firstinspires.ftc.teamcode.Auto.SplineTest;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOp.CoaxialDrive;

@Autonomous(name="Cheezits Turn Test", group="Cheezits")
public class exampleTurn extends LinearOpMode {

    private CoaxialDrive myHardware;
    private Pose2d currentPose;
    private Pose2d targetPose;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and drivetrain
        myHardware = new CoaxialDrive(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); // Wait for the start signal

        // Set start pose (at 0 degrees) and target pose (90-degree rotation)
        currentPose = new Pose2d(0, 0, new Rotation2d(0)); // Facing forward
        targetPose = new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))); // Rotate to 90 degrees

        while (opModeIsActive()) {
            myHardware.MoveRobotLinear(targetPose, currentPose);
            Log.i("MOVING ROBOT ", "TO TARGET ROTATION: " + Math.toDegrees(targetPose.getRotation().getRadians()));
            Log.i("MOVING ROBOT ", "AT CURRENT ROTATION: " + Math.toDegrees(currentPose.getRotation().getRadians()));
        }

        myHardware.stop(); // Stop servos after turning
    }
}