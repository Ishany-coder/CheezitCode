package org.firstinspires.ftc.teamcode.Drive;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.Squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class RobotHardware {
    public double Encoder1PrevTicks = Double.NaN;
    public double Encoder2PrevTicks = Double.NaN;
    public double Encoder3PrevTicks = Double.NaN;
    public double WheelRadius = RobotConstants.WheelRadius;
    public double EncoderTicksPerRev = RobotConstants.EncoderTicksPerRev;
    public double LeftAndRightEncoderDist = RobotConstants.LeftAndRightEncoderDist;
    public double FrontEncoderOffset = RobotConstants.FrontEncoderOffset;
    double ServoDegrees = RobotConstants.ServoDegrees;
    public double MotorSpeed = RobotConstants.MotorSpeed;
    private Servo RightFrontServo;
    private Servo LeftFrontServo;
    private Servo RightBackServo;
    private Servo LeftBackServo;
    private DcMotorEx RightFrontMotor; // Right Front
    private DcMotorEx LeftFrontMotor; // Left Front
    private DcMotorEx RightBackMotor; // Right Back
    private DcMotorEx LeftBackMotor; // Left Back
    private DrivetrainSquIDController squid;

    public RobotHardware(HardwareMap hardwareMap) {
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
    }

    public void turn(double angle) {
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

    public void moveForward() {
        Log.i("MOVING ROBOT: ", "FORWARD");
        if (MotorSpeed < 0) {
            MotorSpeed *= -1;
        }
        //Move motors forward
        RightFrontMotor.setPower(MotorSpeed);
        LeftFrontMotor.setPower(MotorSpeed);
        RightBackMotor.setPower(MotorSpeed);
        LeftBackMotor.setPower(MotorSpeed);
    }

    public void moveBackward() {
        Log.i("MOVING ROBOT: ", "BACKWARD");
        if (MotorSpeed > 0) {
            MotorSpeed *= -1;
        }
        RightFrontMotor.setPower(MotorSpeed);
        LeftFrontMotor.setPower(MotorSpeed);
        RightBackMotor.setPower(MotorSpeed);
        LeftBackMotor.setPower(MotorSpeed);
    }

    public void stop() {
        Log.i("STATUS: ", "STOPPED");
        //Stop motors
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        RightBackMotor.setPower(0);
        LeftBackMotor.setPower(0);
    }
    public Pose2d updatePose(Pose2d currentPose){
        if (Double.isNaN(Encoder1PrevTicks)) {
            Encoder1PrevTicks = RightFrontMotor.getCurrentPosition();
            Encoder2PrevTicks = LeftFrontMotor.getCurrentPosition();
            Encoder3PrevTicks = RightBackMotor.getCurrentPosition();
            return currentPose;
        }
        double DeltaEncoder1Ticks = RightFrontMotor.getCurrentPosition() - Encoder1PrevTicks;
        double DeltaEncoder2Ticks = LeftFrontMotor.getCurrentPosition() - Encoder2PrevTicks;
        double DeltaEncoder3Ticks = RightBackMotor.getCurrentPosition() - Encoder3PrevTicks;
        double I = (2 * Math.PI * WheelRadius) / EncoderTicksPerRev;
        double DeltaX = I * ((DeltaEncoder1Ticks + DeltaEncoder2Ticks) / 2);
        double DeltaTheta = I * (((DeltaEncoder1Ticks - DeltaEncoder2Ticks) / LeftAndRightEncoderDist));
        double DeltaY = I * (DeltaEncoder3Ticks - (FrontEncoderOffset * (DeltaEncoder2Ticks - DeltaEncoder1Ticks) / LeftAndRightEncoderDist));
        double NewX = currentPose.getX() + DeltaX * Math.cos(DeltaTheta) - DeltaY * Math.sin(DeltaTheta); // Apply vector math to find new X
        double NewY = currentPose.getY() + DeltaX * Math.sin(DeltaTheta) + DeltaY * Math.cos(DeltaTheta); // Apply vector math to find new Y
        double NewTheta = currentPose.getHeading() + DeltaTheta; // Apply vector math to find new Theta
        //Update prev encoder ticks
        Encoder1PrevTicks = RightFrontMotor.getCurrentPosition();
        Encoder2PrevTicks = LeftFrontMotor.getCurrentPosition();
        Encoder3PrevTicks = RightBackMotor.getCurrentPosition();
        return new Pose2d(NewX, NewY, new Rotation2d(NewTheta));
    }
    public void moveLinearAndTurn(Pose2d targetPose){
        if (targetPose.getY() > 0) {
            //Move and turn everything forward maybe change sequential to parallel?
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> turn(targetPose.getHeading())),
                            new InstantCommand(() -> moveForward()),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> turn(targetPose.getHeading())),
                                    new InstantCommand(() -> moveBackward())

                            )
                    )
            );
        }
        else if(targetPose.getY() < 0){
            CommandScheduler.getInstance().schedule(
                    //Move and turn everything backward maybe change sequential to parallel?
                    new SequentialCommandGroup(
                            new InstantCommand(() -> turn(targetPose.getHeading())),
                            new InstantCommand(() -> moveBackward()),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> turn(targetPose.getHeading())),
                                    new InstantCommand(() -> moveBackward())

                            )
                    )
            );
        }
    }
    public void MoveRobotLinear(Pose2d targetPose, Pose2d currentPose){
        Pose2d movement = squid.calculate(targetPose, currentPose, new Pose2d(0, 0, new Rotation2d(0)));
        Log.i("MOVING ROBOT: ", "LINEAR MOVEMENT");
        double angle = getAngle(movement.getY(), movement.getX());
        //move forward
        if(movement.getY() > 0){
            Log.i("MOVING ROBOT LINEARLY: ", "FORWARD");
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> turn(angle)),
                            new InstantCommand(() -> moveForward())
                    )
            );
        }
        else if(movement.getY() < 0){
            Log.i("MOVING ROBOT LINEARLY: ", "BACKWARD");
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> turn(angle)),
                            new InstantCommand(() -> moveBackward())
                    )
            );
        }
        if(currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.5){
            Log.i("MOVING ROBOT LINEARLY: ", "STOPPED");
            stop();
        }
    }
    public void MoveSplinePurePursuitWithSquID(
            Pose2d currentPose,
            List<Pose2d> path,
            DrivetrainSquIDController squid,
            RobotHardware drive
    ) {
        Log.i("MOVING ROBOT: ", "PURE PURSUIT STARTED");
        double lookaheadDistance = 6.0;
        double positionTolerance = 1.0;

        int lastClosestIndex = 0;

        while (true) {
            // 0. Update pose estimate
            currentPose = updatePose(currentPose);
            // 1. Find the closest point, starting from last closest index
            Pose2d closestPoint = path.get(lastClosestIndex);
            double closestDistance = Double.MAX_VALUE;

            for (int i = lastClosestIndex; i < path.size(); i++) {
                double dist = currentPose.getTranslation().getDistance(path.get(i).getTranslation());
                if (dist < closestDistance) {
                    closestDistance = dist;
                    closestPoint = path.get(i);
                    lastClosestIndex = i;
                }
            }

            Log.i("CLOSEST POINT:", "X: " + closestPoint.getX() + " Y: " + closestPoint.getY());

            // 2. Find lookahead point at distance from current pose
            Pose2d lookaheadPoint = path.get(path.size() - 1); // default to final
            for (int i = lastClosestIndex; i < path.size(); i++) {
                double dist = currentPose.getTranslation().getDistance(path.get(i).getTranslation());
                if (dist >= lookaheadDistance) {
                    lookaheadPoint = path.get(i);
                    break;
                }
            }

            Log.i("LOOKAHEAD POINT:", "X: " + lookaheadPoint.getX() + " Y: " + lookaheadPoint.getY());

            // 3. Calculate movement using SquID
            Pose2d movement = squid.calculate(lookaheadPoint, currentPose, lookaheadPoint);  // use full pose target

            // 4. Determine angle
            double angle = Math.toDegrees(Math.atan2(movement.getY(), movement.getX()));
            angle = (angle + 360) % 360;

            if (angle > 180) {
                drive.turn(angle - 180);
                drive.moveBackward();
            } else {
                drive.turn(angle);
                drive.moveForward();
            }


            // 6. Check if we're near the final point
            Pose2d finalPoint = path.get(path.size() - 1);
            if (currentPose.getTranslation().getDistance(finalPoint.getTranslation()) < positionTolerance) {
                Log.i("PURE PURSUIT: ", "REACHED FINAL POINT");
                drive.stop();
                break;
            }
        }
    }
}