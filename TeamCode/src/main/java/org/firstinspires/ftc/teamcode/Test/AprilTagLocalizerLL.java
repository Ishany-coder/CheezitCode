package org.firstinspires.ftc.teamcode.Test;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Localization.AprilTag.AprilTagLimelight;

@Autonomous
public class AprilTagLocalizerLL extends LinearOpMode {
    private AprilTagLimelight aprilTag;
    public double x, y, h;
    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag = new AprilTagLimelight(this.hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            Pose2d newPose = aprilTag.calculate(new Pose2d(0,0,new Rotation2d(0))); // calculate from a pose of zero
            x = newPose.getX();
            y = newPose.getY();
            h = newPose.getHeading();
            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.addData("Heading: ", h);
            telemetry.update();
        }
    }
}
