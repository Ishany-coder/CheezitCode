package org.firstinspires.ftc.teamcode.Auto.SplineTest.FTCLib;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drive.GenerateSpline;

import java.util.List;

@Autonomous
public class SplineFTCLIB extends LinearOpMode {
    public GenerateSpline spline;
    private Pose2d currentPose = new Pose2d(0, 0, new Rotation2d(0));
    public Pose2d targetPose = new Pose2d(10, 10, new Rotation2d(0));
    List<Translation2d> midPoints = List.of(
            new Translation2d(2, 2),
            new Translation2d(5, 5)
    );
    @Override
    public void runOpMode() throws InterruptedException {
        Log.i("SPLINE FTC LIB TEST STATUS: ", "INITIAlIZED");
        spline = new GenerateSpline();
        waitForStart();
        while(opModeIsActive()){
            spline.MoveRobotSplineFTClib(targetPose, currentPose, midPoints);
        }
    }
}
