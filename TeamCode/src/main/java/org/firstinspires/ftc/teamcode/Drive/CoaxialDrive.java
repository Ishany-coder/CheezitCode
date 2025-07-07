package org.firstinspires.ftc.teamcode.Drive;

import static org.firstinspires.ftc.teamcode.RobotConstants.MotorSpeed;
import static org.firstinspires.ftc.teamcode.RobotConstants.ServoDegrees;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller.squid.DrivetrainSquIDController;

public class CoaxialDrive extends SubsystemBase {
    private Servo RightFrontServo;
    private Servo LeftFrontServo;
    private Servo RightBackServo;
    private Servo LeftBackServo;
    private DcMotorEx RightFrontMotor; // Right Front
    private DcMotorEx LeftFrontMotor; // Left Front
    private DcMotorEx RightBackMotor; // Right Back
    private DcMotorEx LeftBackMotor; // Left Back
    private DrivetrainSquIDController squid;
    private Odometery odo;

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
        odo = new Odometery(RightFrontMotor, LeftFrontMotor, RightBackMotor, hardwareMap);
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
    // also take into acount if there is any error from the first element of the path what to do
    // Diff veloc for diff splines dis for bezier
//    public void MoveSplineBezier(
//            List<Pose2d> path
//    ) {
//        Pose2d startPose = path.get(0);
//        Log.i("MOVING ROBOT", "PURE PURSUIT STARTED");
//
//        PurePursuitController controller = new PurePursuitController(6.0);
//        double positionTolerance = 1.0;
//
//        while () {
//            // 0. Update pose estimate
//            Pose2d oldStartPose = path.get(0);
//            startPose = odo.updatePose(startPose);
//            // check if the start pose has changed
//            if(oldStartPose.getX() - startPose.getX() > 1 || oldStartPose.getY() - startPose.getY() > 1){
//
//            }
//            // 1. Get lookahead point
//            Pose2d lookahead = controller.getLookaheadPoint(path, startPose);
//
//            Log.i("LOOKAHEAD POINT", "X: " + lookahead.getX() + " Y: " + lookahead.getY());
//
//            // 2. Use SquID to compute motion vector toward lookahead
//            Pose2d movement = squid.calculate(lookahead, startPose, lookahead);
//
//            // 3. Extract heading from movement vector
//            double angle = Math.toDegrees(Math.atan2(movement.getY(), movement.getX()));
//            angle = (angle + 360) % 360;
//
//            // 4. Move based on heading
//            if (angle > 180) {
//                turn(angle - 180);
//                moveBackward();
//            } else {
//                turn(angle);
//                moveForward();
//            }
//
//            // 5. Stop if close to final goal
//            if (controller.isFinished(path, startPose, positionTolerance)) {
//                Log.i("PURE PURSUIT", "REACHED FINAL POINT");
//                stop();
//                break;
//            }
//        }
//    }
    //calculate vel later js use var for now

    public void Strafe(boolean Right){
        turn(90);
        if(Right){
            moveForward();
        }
        else{
            moveBackward();
        }
    }
}