package org.firstinspires.ftc.teamcode.Auto.SplineTest.Custom;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.teamcode.Controller.squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Drive.GenerateSpline;
import org.firstinspires.ftc.teamcode.Drive.RobotHardware;

@Autonomous
public class CustomSplineMovementQuintic extends LinearOpMode {
    public RobotHardware drive;
    public GenerateSpline spline;
    public DrivetrainSquIDController squIDController;
    private Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d endPose = new Pose2d(10,10, new Rotation2d(0));
    private Translation2d startVel = new Translation2d(1,1);
    private Translation2d endVel = new Translation2d(1,1);
    private Translation2d startaccel = new Translation2d(10,10);
    private Translation2d endaccel = new Translation2d(10,10);
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new RobotHardware(hardwareMap);
        spline = new GenerateSpline();
        squIDController = new DrivetrainSquIDController();
        waitForStart();
        while(opModeIsActive()){
            List<Pose2d> path = spline.generateQuinticSpline(startPose, startVel, startaccel ,endPose, endVel, endaccel);
            drive.MoveSpline(startPose, path, squIDController);
        }
    }
}
