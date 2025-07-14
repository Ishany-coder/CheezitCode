package org.firstinspires.ftc.teamcode.Drive;

import static org.firstinspires.ftc.teamcode.RobotConstants.MotorSpeed;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.Controller.FeedForward.FeedforwardController;
import org.firstinspires.ftc.teamcode.Controller.PurePursuit.PurePursuitController;
import org.firstinspires.ftc.teamcode.Controller.squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Localization.Odo.Odometery;

import com.arcrobotics.ftclib.geometry.Pose2d;

import java.util.List;
@Config

public class MecanumDriveSubsystem extends SubsystemBase {

    private static DcMotorEx rightFront = null;
    private static DcMotorEx rightBack = null;
    private static DcMotorEx leftFront = null;
    private static DcMotorEx leftBack = null;
    private static DrivetrainSquIDController squid = new DrivetrainSquIDController();
    private static Odometery odo = null;
    public static double KV = 0.1;
    public static double KA = 0.1;

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

    public static void drive(double forward, double strafe, double rotation) {
        double rf = forward - strafe - rotation;
        double rb = forward + strafe - rotation;
        double lf = forward + strafe + rotation;
        double lb = forward - strafe + rotation;
        if(rf>1){
            rf=1;
        }
        if(rb>1){
            rb=1;
        }
        if(lf>1){
            lf=1;
        }
        if(lb>1){
            lb=1;
        }
        rightFront.setPower(rf);
        rightBack.setPower(rb);
        leftFront.setPower(lf);
        leftBack.setPower(lb);

        Log.i("MECANUM POWER", "RF: " + rf + " LF: " + lf + " RB: " + rb + " LB: " + lb);
    }

    public static void stop() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
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
    public void MoveSplineWithSplineHeading(List<Pose2d> path, Double maxVel, Double accel, Double decel) {
        if (path.isEmpty()) return;

        Pose2d prevPose = path.get(0);
        long prevTime = System.currentTimeMillis();
        FeedforwardController ff = new FeedforwardController(KV, KA);
        Log.i("MOVING ROBOT", "INITIALIZED");

        double currentVel = 0;
        double RemainingDist = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            Pose2d expectedPose = path.get(i);
            Pose2d currentPose = odo.updatePose(prevPose);

            // Calculate dt
            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - prevTime) / 1000.0; // seconds
            prevTime = currentTime;

            // Calculate current velocity vector from odometry
            Pose2d velocityVector = new Pose2d(
                    (currentPose.getX() - prevPose.getX()) / dt,
                    (currentPose.getY() - prevPose.getY()) / dt,
                    new Rotation2d((currentPose.getHeading() - prevPose.getHeading()) / dt)
            );

            // Error check
            double dx = expectedPose.getX() - currentPose.getX();
            double dy = expectedPose.getY() - currentPose.getY();
            double distanceError = Math.hypot(dx, dy);

            // If off path, use SquID to correct
            if (distanceError > 0.5) {
                int start = Math.max(0, i - 10);
                int end = Math.min(path.size(), i + 10);

                Pose2d closestPose = path.get(i);
                double closestDistance = currentPose.getTranslation().getDistance(closestPose.getTranslation());

                for (int j = start; j < end; j++) {
                    Pose2d candidate = path.get(j);
                    double dist = currentPose.getTranslation().getDistance(candidate.getTranslation());
                    if (dist < closestDistance) {
                        closestPose = candidate;
                        closestDistance = dist;
                    }
                }

                Pose2d correction = squid.calculate(closestPose, currentPose, velocityVector);
                driveToPose(correction, currentPose, velocityVector);
            }

            // Get lookahead point
            PurePursuitController controller = new PurePursuitController(6);
            Pose2d lookahead = controller.getLookaheadPoint(path, currentPose);
            Log.i("LOOKAHEAD POINT", "X: " + lookahead.getX() + " Y: " + lookahead.getY());

            // Get motion vector using SquID
            Pose2d movement = squid.calculate(lookahead, currentPose, velocityVector);

            // Calculate target velocity (trapezoidal motion profile)
            if (maxVel != null && accel != null && decel != null) {
                Pose2d a = path.get(i);
                Pose2d b = path.get(i + 1);
                RemainingDist += a.getTranslation().getDistance(b.getTranslation());
                double stoppingDistance = (currentVel * currentVel) / (2 * decel);
                if (stoppingDistance >= RemainingDist) {
                    currentVel -= decel * dt;
                } else {
                    currentVel += accel * dt;
                }
                currentVel = Math.min(currentVel, maxVel);
            } else {
                currentVel = 1.0; // Default constant speed
            }
            // Compute acceleration estimates
            double ax = (velocityVector.getX() - (currentPose.getX() - prevPose.getX()) / dt) / dt;
            double ay = (velocityVector.getY() - (currentPose.getY() - prevPose.getY()) / dt) / dt;

            // Feedforward power commands
            double powerX = ff.calculate(movement.getX(), ax);
            double powerY = ff.calculate(movement.getY(), ay);

            // Normalize and scale to current velocity
            // norm linear dist left
            double norm = Math.hypot(powerX, powerY);
            double ffx = 0;
            double ffy = 0;
            if (norm > 0.01) {
                ffx = (powerX / norm) * currentVel;
                ffy = (powerY / norm) * currentVel;
            }
            driveToPose(new Pose2d(
                    ffx + movement.getX(),
                    ffy + movement.getY(),
                    new Rotation2d(movement.getHeading())),
                    currentPose,
                    velocityVector
            );
            prevPose = currentPose;
        }
        Log.i("MOVING ROBOT", "PURE PURSUIT FINISHED");
    }
    public void driveToPose(Pose2d targetPose, Pose2d currentPose, Pose2d velocityVector) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double dt = targetPose.getHeading() - currentPose.getHeading();

        long startTime = System.currentTimeMillis();

        while (Math.abs(dx) >= 0.01 || Math.abs(dy) >= 0.01 || Math.abs(dt) >= 0.01) {
            // Exit if it takes too long
            if (System.currentTimeMillis() - startTime >= 2000) break;

            currentPose = odo.updatePose(currentPose);

            drive(velocityVector.getY(), velocityVector.getX(), velocityVector.getHeading());

            dx = targetPose.getX() - currentPose.getX();
            dy = targetPose.getY() - currentPose.getY();
            dt = targetPose.getHeading() - currentPose.getHeading();
        }

        stop(); // Optional — stop once you reach target
    }
}