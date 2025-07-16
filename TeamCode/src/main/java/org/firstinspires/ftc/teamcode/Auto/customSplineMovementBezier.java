package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controller.squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial.SplineCommand;
import org.firstinspires.ftc.teamcode.Drive.GenerateSpline;
import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;

import java.util.List;

@Autonomous
public class customSplineMovementBezier extends LinearOpMode {
    public CoaxialDrive drive;
    public GenerateSpline spline;
    public DrivetrainSquIDController squid;
    private List<Pose2d> path;
    private Translation2d control1 = new Translation2d(5, 5);
    private Translation2d control2 = new Translation2d(5, 5);
    private Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d endPose = new Pose2d(10,10, new Rotation2d(0));
    @Override
    public void runOpMode() throws InterruptedException {
        double vel = 1;
        double accel = 1;
        double decel = 1;
        spline = new GenerateSpline();
        squid = new DrivetrainSquIDController();
        waitForStart();
        while(opModeIsActive()){
            path = spline.makeBezierWithHeadingV2(startPose, endPose, control1, control2); // get a list of points in path
            new SplineCommand(path, vel, accel, decel);
        }
    }
}
