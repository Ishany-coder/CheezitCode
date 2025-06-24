package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public CoaxialDrive(HardwareMap hardwareMap) {
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
        Log.i("GOT ANGLE TO TURN SERVOS: ", "ANGLE: " + Math.toDegrees(Math.atan2(ypos, xpos)) + " degrees")
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
}