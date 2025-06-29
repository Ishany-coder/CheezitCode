package org.firstinspires.ftc.teamcode.Auto.SplineTest.Custom;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controller.squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial.SplineCommand;
import org.firstinspires.ftc.teamcode.Drive.GenerateSpline;
import org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial.CoaxialDrive;

import java.util.List;

@Autonomous
public class customSplineMovementHermite extends LinearOpMode {
    public CoaxialDrive drive;
    public GenerateSpline spline;
    public DrivetrainSquIDController squid;
    private List<Pose2d> path;
    private double startMag = 3;
    private double endMag = 3;
    private Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d endPose = new Pose2d(10,10, new Rotation2d(0));
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new CoaxialDrive(hardwareMap);
        spline = new GenerateSpline();
        squid = new DrivetrainSquIDController();
        waitForStart();
        while(opModeIsActive()){
            path = spline.generateHermiteCurve(startPose, startMag, endPose, endMag); // get a list of points in path
            new SplineCommand(drive, path);
        }
    }
}
