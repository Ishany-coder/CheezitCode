package org.firstinspires.ftc.teamcode.Test;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Localization.AprilTag.AprilTagOpenCV;
import org.firstinspires.ftc.teamcode.Localization.AprilTag.AprilTagOpenCV;

import java.util.Collections;
import java.util.List;

@Autonomous
public class AprilTagLocalizerOpenCv extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d currentPose = new Pose2d(0,0,new Rotation2d(0));
        Pose2d newPose;
        List<Pose2d> possiblePoses = Collections.emptyList();
        AprilTagOpenCV aprilTag = new AprilTagOpenCV(this.telemetry, this.hardwareMap);
        while(opModeIsActive()){
            possiblePoses = aprilTag.updatePose(currentPose);
            newPose = possiblePoses.get(0);
            telemetry.addData("Got Pose X: ", newPose.getX());
            telemetry.addData("Got Pose Y: ", newPose.getY());
            telemetry.addData("Got Pose Theta: ", newPose.getRotation().getDegrees());
            telemetry.update();
        }
    }
}
