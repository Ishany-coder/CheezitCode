package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//change the code so at certain points it can use 1 motor to turn and move the wheel simultaneously

public class SwerveDrive {
    private final int GearRatio = 8;
    private Servo topLeftServo1;
    private Servo topLeftServo2;
    private Servo topRightServo1;
    private Servo topRightServo2;
    private Servo bottomLeftServo1;
    private Servo bottomLeftServo2;
    private Servo bottomRightServo1;
    private Servo bottomRightServo2;

    public SwerveDrive(HardwareMap hardwareMap) {
        topLeftServo1 = hardwareMap.get(Servo.class, "topLeftServo1");
        topLeftServo2 = hardwareMap.get(Servo.class, "topLeftServo2");
        topRightServo1 = hardwareMap.get(Servo.class, "topRightServo1");
        topRightServo2 = hardwareMap.get(Servo.class, "topRightServo2");
        bottomLeftServo1 = hardwareMap.get(Servo.class, "bottomLeftServo1");
        bottomLeftServo2 = hardwareMap.get(Servo.class, "bottomLeftServo2");
        bottomRightServo1 = hardwareMap.get(Servo.class, "bottomRightServo1");
        bottomRightServo2 = hardwareMap.get(Servo.class, "bottomRightServo2");
    }

    public void turn(double turn) {
        double adjustedTurn = turn * GearRatio;

        int wholeRotations = (int) adjustedTurn;  // Number of whole rotations
        double fractionalRotation = adjustedTurn - wholeRotations; // Remaining fractional rotation

        for (int i = 0; i < wholeRotations; i++) {
            setServoPositions(1, 1, 1, 1, 1, 1, 1, 1);
            delay(50); // Allow servos time to complete each rotation
            setServoPositions(0, 0, 0, 0, 0, 0, 0, 0);
        }

        if (fractionalRotation > 0) {
            setServoPositions(fractionalRotation, fractionalRotation, fractionalRotation, fractionalRotation,
                    fractionalRotation, fractionalRotation, fractionalRotation, fractionalRotation);
        }
    }

    public double getAngle(double ypos, double xpos) {
        return (Math.atan2(ypos, xpos)) / Math.PI;
    }

    public void turnDriveMotors(double angle) {
        double adjustedAngle = angle * GearRatio;

        int wholeRotations = (int) adjustedAngle;
        double fractionalRotation = adjustedAngle - wholeRotations;

        for (int i = 0; i < wholeRotations; i++) {
            setServoPositions(1, 1, 1, 1, 1, 1, 1, 1);
            delay(50);
            setServoPositions(0, 0, 0, 0, 0, 0, 0, 0);
        }

        if (fractionalRotation > 0) {
            setServoPositions(fractionalRotation, fractionalRotation, fractionalRotation, fractionalRotation,
                    fractionalRotation, fractionalRotation, fractionalRotation, fractionalRotation);
        }
    }

    public void moveForward() {
        // move Forward
        bottomLeftServo1.setDirection(Servo.Direction.REVERSE);
        topLeftServo1.setDirection(Servo.Direction.REVERSE);
        bottomRightServo1.setDirection(Servo.Direction.REVERSE);
        topRightServo1.setDirection(Servo.Direction.REVERSE);
        ContinouslyRotate();
    }

    public void moveBackward() {
        // move Backward
        bottomLeftServo2.setDirection(Servo.Direction.REVERSE);
        topLeftServo2.setDirection(Servo.Direction.REVERSE);
        bottomRightServo2.setDirection(Servo.Direction.REVERSE);
        topRightServo2.setDirection(Servo.Direction.REVERSE);
        ContinouslyRotate();
    }

    public void stopServos() {
        setServoPositions(topLeftServo1.getPosition(), topLeftServo2.getPosition(),
                topRightServo1.getPosition(), topRightServo2.getPosition(),
                bottomLeftServo1.getPosition(), bottomLeftServo2.getPosition(),
                bottomRightServo1.getPosition(), bottomRightServo2.getPosition());
    }

    private void setServoPositions(double tl1, double tl2, double tr1, double tr2,
                                   double bl1, double bl2, double br1, double br2) {
        topLeftServo1.setPosition(tl1);
        topLeftServo2.setPosition(tl2);
        topRightServo1.setPosition(tr1);
        topRightServo2.setPosition(tr2);
        bottomLeftServo1.setPosition(bl1);
        bottomLeftServo2.setPosition(bl2);
        bottomRightServo1.setPosition(br1);
        bottomRightServo2.setPosition(br2);
    }

    private void ContinouslyRotate() {
        if (topLeftServo1.getPosition() >= 1 || topLeftServo2.getPosition() >= 1 ||
                topRightServo1.getPosition() >= 1 || topRightServo2.getPosition() >= 1 ||
                bottomLeftServo1.getPosition() >= 1 || bottomLeftServo2.getPosition() >= 1 ||
                bottomRightServo1.getPosition() >= 1 || bottomRightServo2.getPosition() >= 1) {

            topLeftServo1.setPosition(0);
            topLeftServo2.setPosition(0);
            topRightServo1.setPosition(0);
            topRightServo2.setPosition(0);
            bottomLeftServo1.setPosition(0);
            bottomLeftServo2.setPosition(0);
            bottomRightServo1.setPosition(0);
            bottomRightServo2.setPosition(0);
        } else {
            topLeftServo1.setPosition(topLeftServo1.getPosition() + 0.1);
            topLeftServo2.setPosition(topLeftServo2.getPosition() + 0.1);
            topRightServo1.setPosition(topRightServo1.getPosition() + 0.1);
            topRightServo2.setPosition(topRightServo2.getPosition() + 0.1);
            bottomLeftServo1.setPosition(bottomLeftServo1.getPosition() + 0.1);
            bottomLeftServo2.setPosition(bottomLeftServo2.getPosition() + 0.1);
            bottomRightServo1.setPosition(bottomRightServo1.getPosition() + 0.1);
            bottomRightServo2.setPosition(bottomRightServo2.getPosition() + 0.1);
        }
    }

    private void delay(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}