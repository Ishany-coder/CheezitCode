package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.TeleOp.CoaxialDrive;

org.firstinspires.ftc.teamcode.Auto.DrivetrainSquIDController
@Autonomous(name = "SquID Tuner", group = "Cheezits")
public class SquIDTuner extends LinearOpMode {

    public static double looptimeAdjuster = 15;

    private CoaxialDrive drive;
    private org.firstinspires.ftc.teamcode.Auto.DrivetrainSquIDController squidController;
    private Pose2d currentPose, targetPose;
    private ElapsedTime timer;

    @Override
    public void runOpMode() {
        drive = new Drive(this.hardwareMap);
        squidController = new DrivetrainSquIDController();
        timer = new ElapsedTime();

        currentPose = new Pose2d(0, 0, new Rotation2d(0));
        targetPose = new Pose2d(24, 0, new Rotation2d(0));

        waitForStart();

        boolean movingForward = true;

        while (opModeIsActive()) {
            Pose2d movement = squidController.calculate(targetPose, currentPose, new Pose2d(0, 0, new Rotation2d(0)));

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
}