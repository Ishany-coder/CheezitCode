package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import static org.firstinspires.ftc.teamcode.RobotConstants.MotorSpeed;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.Controller.PurePursuit.PurePursuitController;
import org.firstinspires.ftc.teamcode.Controller.squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Drive.Odometery;
import com.arcrobotics.ftclib.geometry.Pose2d;

import java.util.List;

public class MecanumDriveSubsystem extends SubsystemBase {

    private final DcMotorEx rightFront, rightBack, leftFront, leftBack;
    private final DrivetrainSquIDController squid;
    private final Odometery odo;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFrontMotor");
        rightBack  = hardwareMap.get(DcMotorEx.class, "RightBackMotor");
        leftFront  = hardwareMap.get(DcMotorEx.class, "LeftFrontMotor");
        leftBack   = hardwareMap.get(DcMotorEx.class, "LeftBackMotor");

        DcMotorEx[] motors = {rightFront, rightBack, leftFront, leftBack};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        squid = new DrivetrainSquIDController();
        odo = new Odometery(rightFront, leftFront, rightBack, hardwareMap);
        Log.i("MECANUM DRIVE", "Initialized");
    }

    public void drive(double forward, double strafe, double rotation) {
        double rf = forward - strafe - rotation;
        double rb = forward + strafe - rotation;
        double lf = forward + strafe + rotation;
        double lb = forward - strafe + rotation;

        rightFront.setPower(rf);
        rightBack.setPower(rb);
        leftFront.setPower(lf);
        leftBack.setPower(lb);

        Log.i("MECANUM POWER", "RF: " + rf + " LF: " + lf + " RB: " + rb + " LB: " + lb);
    }

    public void stop() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }

    public void driveToPose(Pose2d targetPose) {
        Pose2d currentPose = odo.updatePose(new Pose2d());
        Pose2d movement = squid.calculate(targetPose, currentPose, new Pose2d(0, 0, new com.arcrobotics.ftclib.geometry.Rotation2d()));

        drive(movement.getY(), movement.getX(), movement.getHeading());
    }

    public void followPath(List<Pose2d> path) {
        PurePursuitController controller = new PurePursuitController(6.0);
        Pose2d pose = odo.updatePose(new Pose2d());

        while (!controller.isFinished(path, pose, 1.0)) {
            pose = odo.updatePose(pose);
            Pose2d lookahead = controller.getLookaheadPoint(path, pose);
            Pose2d movement = squid.calculate(lookahead, pose, lookahead);
            drive(movement.getY(), movement.getX(), 0);
        }

        stop();
    }

    public Pose2d getCurrentPose() {
        return odo.updatePose(new Pose2d());
    }

    public void moveForward() {
        Log.i("MOVING ROBOT: ", "FORWARD");
        if (MotorSpeed < 0) {
            MotorSpeed *= -1;
        }
        //Move motors forward
        rightFront.setPower(MotorSpeed);
        leftFront.setPower(MotorSpeed);
        leftBack.setPower(MotorSpeed);
        rightBack.setPower(MotorSpeed);
    }

    public void moveBackward() {
        Log.i("MOVING ROBOT: ", "BACKWARD");
        if (MotorSpeed > 0) {
            MotorSpeed *= -1;
        }
        rightFront.setPower(MotorSpeed);
        leftFront.setPower(MotorSpeed);
        leftBack.setPower(MotorSpeed);
        rightBack.setPower(MotorSpeed);
    }
}