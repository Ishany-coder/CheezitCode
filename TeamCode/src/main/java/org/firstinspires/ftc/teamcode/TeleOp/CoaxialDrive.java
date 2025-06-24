package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.DrivetrainSquIDController;

//change the code so at certain points it can use 1 motor to turn and move the wheel simultaneously
@Config
public class CoaxialDrive {
    public double ServoDegrees = 300;
    public static double MotorSpeed = 1;
    private Servo ServoPod1;
    private Servo ServoPod2;
    private Servo ServoPod3;
    private Servo ServoPod4;
    private DcMotorEx MotorPod1;
    private DcMotorEx MotorPod2;
    private DcMotorEx MotorPod3;
    private DcMotorEx MotorPod4;
    private DrivetrainSquIDController squid;

    public CoaxialDrive(HardwareMap hardwareMap) {
        squid = new DrivetrainSquIDController();
        Log.i("STATUS: ", "INITIALIZED");
        //Initialize motors and their behaviors
        MotorPod1 = hardwareMap.get(DcMotorEx.class, "MotorPod1");
        MotorPod1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorPod1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorPod2 = hardwareMap.get(DcMotorEx.class, "MotorPod2");
        MotorPod2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorPod2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorPod3 = hardwareMap.get(DcMotorEx.class, "MotorPod3");
        MotorPod3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorPod3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorPod4 = hardwareMap.get(DcMotorEx.class, "MotorPod4");
        MotorPod4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorPod4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Initialize servos
        ServoPod1 = hardwareMap.get(Servo.class, "ServoPod1");
        ServoPod2 = hardwareMap.get(Servo.class, "ServoPod2");
        ServoPod3 = hardwareMap.get(Servo.class, "ServoPod3");
        ServoPod4 = hardwareMap.get(Servo.class, "ServoPod4");
    }
    public void turn(double angle) {
        Log.i("TURNING SERVOS ", "ANGLE: " + angle);
        //Move Right
        if(ServoPod1.getPosition()*ServoDegrees > angle){
            Log.i("TURNING SERVOS: ", "RIGHT");
            //Move Servos
            ServoPod1.setPosition(angle/ServoDegrees);
            ServoPod2.setPosition(angle/ServoDegrees);
            ServoPod3.setPosition(angle/ServoDegrees);
            ServoPod4.setPosition(angle/ServoDegrees);
        }
        //Move Left
        else if(ServoPod1.getPosition() * ServoDegrees < angle){
            Log.i("TURNING SERVOS: ", "LEFT");
            //Set the direction to reverse or left then move
            ServoPod1.setDirection(Servo.Direction.REVERSE);
            ServoPod2.setDirection(Servo.Direction.REVERSE);
            ServoPod3.setDirection(Servo.Direction.REVERSE);
            ServoPod4.setDirection(Servo.Direction.REVERSE);
            //Move Servos
            ServoPod1.setPosition(angle/ServoDegrees);
            ServoPod2.setPosition(angle/ServoDegrees);
            ServoPod3.setPosition(angle/ServoDegrees);
            ServoPod4.setPosition(angle/ServoDegrees);
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
        //Move motors forward
        MotorPod1.setPower(MotorSpeed);
        MotorPod2.setPower(MotorSpeed);
        MotorPod3.setPower(MotorSpeed);
        MotorPod4.setPower(MotorSpeed);
    }

    public void moveBackward() {
        Log.i("MOVING ROBOT: ", "BACKWARD");
        // move Backward
        MotorPod1.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorPod2.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorPod3.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorPod4.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorPod1.setPower(MotorSpeed);
        MotorPod2.setPower(MotorSpeed);
        MotorPod3.setPower(MotorSpeed);
        MotorPod4.setPower(MotorSpeed);
    }
    public void stop(){
        Log.i("STATUS: ", "STOPPED");
        //Stop motors
        MotorPod1.setPower(0);
        MotorPod2.setPower(0);
        MotorPod3.setPower(0);
        MotorPod4.setPower(0);
    }
    //function to move robot linearly to a point
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
}