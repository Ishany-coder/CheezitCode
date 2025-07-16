package org.firstinspires.ftc.teamcode.Drive;

import static org.firstinspires.ftc.teamcode.RobotConstants.MotorSpeed;
import static org.firstinspires.ftc.teamcode.RobotConstants.ServoDegrees;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller.FeedForward.FeedforwardController;
import org.firstinspires.ftc.teamcode.Controller.PurePursuit.PurePursuitController;
import org.firstinspires.ftc.teamcode.Controller.squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Localization.Odo.Odometery;

import java.util.List;

public class CoaxialDrive extends SubsystemBase {
    private static Servo RightFrontServo;
    private static Servo LeftFrontServo;
    private static Servo RightBackServo;
    private static Servo LeftBackServo;
    private static DcMotorEx RightFrontMotor; // Right Front
    private static DcMotorEx LeftFrontMotor; // Left Front
    private static DcMotorEx RightBackMotor; // Right Back
    private static DcMotorEx LeftBackMotor; // Left Back
    private DrivetrainSquIDController squid;
    private Odometery odo;
    public static double KV = 0.1;
    public static double KA = 0.1;

    public CoaxialDrive(HardwareMap hardwareMap) {
        squid = new DrivetrainSquIDController();
        Log.i("DRIVE TRAIN STATUS: ", "INITIALIZED");
        //Initialize motors and their behaviors
        RightFrontMotor = hardwareMap.get(DcMotorEx.class, "RightFrontMotor");
        RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftFrontMotor = hardwareMap.get(DcMotorEx.class, "LeftFrontMotor");
        LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightBackMotor = hardwareMap.get(DcMotorEx.class, "RightBackMotor");
        RightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftBackMotor = hardwareMap.get(DcMotorEx.class, "LeftBackMotor");
        LeftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Initialize servos
        RightFrontServo = hardwareMap.get(Servo.class, "RightFrontServo");
        LeftFrontServo = hardwareMap.get(Servo.class, "LeftFrontServo");
        RightBackServo = hardwareMap.get(Servo.class, "RightBackServo");
        LeftBackServo = hardwareMap.get(Servo.class, "LeftBackServo");
        odo = new Odometery(hardwareMap);
    }

    public static void turn(double angle) {
        Log.i("TURNING SERVOS ", "ANGLE: " + angle);
        //Move Right
        if (RightFrontServo.getPosition() * ServoDegrees > angle) {
            Log.i("TURNING SERVOS: ", "RIGHT");
            RightFrontServo.setDirection(Servo.Direction.FORWARD);
            LeftFrontServo.setDirection(Servo.Direction.FORWARD);
            RightBackServo.setDirection(Servo.Direction.FORWARD);
            LeftBackServo.setDirection(Servo.Direction.FORWARD);
            //Move Servos
            RightFrontServo.setPosition(angle / ServoDegrees);
            LeftFrontServo.setPosition(angle / ServoDegrees);
            RightBackServo.setPosition(angle / ServoDegrees);
            LeftBackServo.setPosition(angle / ServoDegrees);
        }
        //Move Left
        else if (RightFrontServo.getPosition() * ServoDegrees < angle) {
            Log.i("TURNING SERVOS: ", "LEFT");
            //Set the direction to reverse or left then move
            RightFrontServo.setDirection(Servo.Direction.REVERSE);
            LeftFrontServo.setDirection(Servo.Direction.REVERSE);
            RightBackServo.setDirection(Servo.Direction.REVERSE);
            LeftBackServo.setDirection(Servo.Direction.REVERSE);
            //Move Servos
            RightFrontServo.setPosition(angle / ServoDegrees);
            LeftFrontServo.setPosition(angle / ServoDegrees);
            RightBackServo.setPosition(angle / ServoDegrees);
            LeftBackServo.setPosition(angle / ServoDegrees);
        }
    }

    public double getAngle(double ypos, double xpos) {
        Log.i("GETTING ANGLE TO MOVE SERVOS: ", "X: " + xpos + " Y: " + ypos);
        //Given an input of x and y pos from controller return angle formed
        Log.i("GOT ANGLE TO TURN SERVOS: ", "ANGLE: " + Math.toDegrees(Math.atan2(ypos, xpos)) + " degrees");
        return (Math.toDegrees(Math.atan2(ypos, xpos)));
    }

    public void moveForward(double speed) {
        Log.i("MOVING ROBOT: ", "FORWARD");
        if (MotorSpeed < 0) {
            MotorSpeed *= -1;
        }
        //Move motors forward
        RightFrontMotor.setPower(speed);
        LeftFrontMotor.setPower(speed);
        RightBackMotor.setPower(speed);
        LeftBackMotor.setPower(speed);
    }

    public void moveBackward(double speed) {
        Log.i("MOVING ROBOT: ", "BACKWARD");
        if (MotorSpeed > 0) {
            MotorSpeed *= -1;
        }
        RightFrontMotor.setPower(speed);
        LeftFrontMotor.setPower(speed);
        RightBackMotor.setPower(speed);
        LeftBackMotor.setPower(speed);
    }

    public static void stop() {
        Log.i("STATUS: ", "STOPPED");
        //Stop motors
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        RightBackMotor.setPower(0);
        LeftBackMotor.setPower(0);
    }
    public void MoveRobotLinear(Pose2d targetPose, Pose2d currentPose, double velocity){
        Pose2d movement = squid.calculate(targetPose, currentPose, new Pose2d(0, 0, new Rotation2d(0)));
        Log.i("MOVING ROBOT: ", "LINEAR MOVEMENT");
        double angle = getAngle(movement.getY(), movement.getX());
        //move forward
        if(movement.getY() > 0){
            Log.i("MOVING ROBOT LINEARLY: ", "FORWARD");
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> turn(angle)),
                            new InstantCommand(() -> moveForward(velocity))
                    )
            );
        }
        else if(movement.getY() < 0){
            Log.i("MOVING ROBOT LINEARLY: ", "BACKWARD");
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> turn(angle)),
                            new InstantCommand(() -> moveBackward(velocity))
                    )
            );
        }
        if(currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.5){
            Log.i("MOVING ROBOT LINEARLY: ", "STOPPED");
            stop();
        }
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
                DriveToPose(correction, currentPose, velocityVector);
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
            DriveToPose(new Pose2d(
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
    public void DriveToPose(Pose2d targetPose, Pose2d currentPose, Pose2d velocityVector){
        // Tuning constants
        double kP_translation = 0.5;
        double kP_rotation = 1.0;

        // Error between current and target pose
        double errorX = targetPose.getX() - currentPose.getX();
        double errorY = targetPose.getY() - currentPose.getY();
        double errorHeading = targetPose.getHeading() - currentPose.getHeading();
        errorHeading = Math.atan2(Math.sin(errorHeading), Math.cos(errorHeading)); // normalize

        // Adjusted drive signals
        double forward = velocityVector.getX() + kP_translation * errorX;
        double strafe = velocityVector.getY() + kP_translation * errorY;
        double turn = velocityVector.getHeading() + kP_rotation * errorHeading;

        // Robot geometry
        drive(forward, strafe, turn);
    }
    public static void drive(double forward, double strafe, double turn) {
        double L = 13.0; // length of robot in inches
        double W = 13.0; // width of robot in inches
        double r = Math.hypot(L, W);

        // Calculate intermediate terms for turning
        double a = strafe - turn * (L / r);
        double b = strafe + turn * (L / r);
        double c = forward - turn * (W / r);
        double d = forward + turn * (W / r);

        // Calculate wheel speeds
        double flSpeed = Math.hypot(b, d);
        double frSpeed = Math.hypot(b, c);
        double blSpeed = Math.hypot(a, d);
        double brSpeed = Math.hypot(a, c);

        // Calculate wheel angles (radians)
        double flAngle = Math.atan2(b, d);
        double frAngle = Math.atan2(b, c);
        double blAngle = Math.atan2(a, d);
        double brAngle = Math.atan2(a, c);

        // Normalize speeds to [0, 1]
        double max = Math.max(Math.max(flSpeed, frSpeed), Math.max(blSpeed, brSpeed));
        if (max > 1.0) {
            flSpeed /= max;
            frSpeed /= max;
            blSpeed /= max;
            brSpeed /= max;
        }

        // Set power
        LeftFrontMotor.setPower(flSpeed);
        RightFrontMotor.setPower(frSpeed);
        LeftBackMotor.setPower(blSpeed);
        RightBackMotor.setPower(brSpeed);

        // Set servo angles (normalized to [0, 1] for -π to π)
        LeftFrontServo.setPosition((flAngle + Math.PI) / (2 * Math.PI));
        RightFrontServo.setPosition((frAngle + Math.PI) / (2 * Math.PI));
        LeftBackServo.setPosition((blAngle + Math.PI) / (2 * Math.PI));
        RightBackServo.setPosition((brAngle + Math.PI) / (2 * Math.PI));
    }
    public void Strafe(boolean Right, double velocity){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> turn(90)),
                        new InstantCommand(() -> moveForward(Right ? velocity : -velocity))
                )
        );
    }
}