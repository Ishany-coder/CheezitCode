package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.DrivetrainSquIDController;

import java.util.ArrayList;
import java.util.List;

//change the code so at certain points it can use 1 motor to turn and move the wheel simultaneously
@Config
public class CoaxialDrive {
    public static double maxVelocitySpline = 10;
    public static double maxAccelSpline = 10;
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
        Log.i("DRIVE TRAIN STATUS: ", "INITIALIZED");
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
    //Use ftc lib spline squid implementation later
    public void MoveRobotSplineFTClib(Pose2d targetPose, Pose2d currentPose, List<Translation2d> midPoints){
        //way points like midpoints for spline
        Log.i("MOVING ROBOT: ", "SPLINE MOVEMENT FTCLIB");
        //Create config
        TrajectoryConfig config = new TrajectoryConfig(maxVelocitySpline, maxAccelSpline);
        //Make spline w/ ftc lib
        TrajectoryGenerator.generateTrajectory(
            currentPose,
            midPoints,
            targetPose,
            config
        );
        Log.i("SPLINE MOVEMENT: ", "DONE");
        stop();
    }

    public static List<Pose2d> generateCubicSplinePath(Pose2d startPose, List<Translation2d> interiorPoints, Pose2d endPose) {
        // Combine all points: start, interior, end
        List<Translation2d> points = new ArrayList<>();
        points.add(startPose.getTranslation());
        points.addAll(interiorPoints);
        points.add(endPose.getTranslation());
        int numSamples = interiorPoints.size();

        int pointCount = points.size();
        double[] t = new double[pointCount];
        for (int i = 0; i < pointCount; i++) t[i] = i;

        // Extract x and y coordinates
        double[] xValues = new double[pointCount];
        double[] yValues = new double[pointCount];
        for (int i = 0; i < pointCount; i++) {
            xValues[i] = points.get(i).getX();
            yValues[i] = points.get(i).getY();
        }

        // Compute second derivatives for x and y
        double[] xSecondDerivatives = computeSecondDerivatives(t, xValues);
        double[] ySecondDerivatives = computeSecondDerivatives(t, yValues);

        List<Pose2d> path = new ArrayList<>();

        for (int sample = 0; sample <= numSamples; sample++) {
            double tSample = (pointCount - 1) * sample / (double)numSamples;
            int segmentIndex = Math.min((int)Math.floor(tSample), pointCount - 2);
            double segmentStart = t[segmentIndex];
            double segmentEnd = t[segmentIndex + 1];
            double segmentLength = segmentEnd - segmentStart;

            double weightStart = (segmentEnd - tSample) / segmentLength;
            double weightEnd = (tSample - segmentStart) / segmentLength;

            double xPos = weightStart * xValues[segmentIndex] + weightEnd * xValues[segmentIndex + 1]
                    + ((Math.pow(weightStart, 3) - weightStart) * xSecondDerivatives[segmentIndex]
                    + (Math.pow(weightEnd, 3) - weightEnd) * xSecondDerivatives[segmentIndex + 1])
                    * Math.pow(segmentLength, 2) / 6.0;

            double yPos = weightStart * yValues[segmentIndex] + weightEnd * yValues[segmentIndex + 1]
                    + ((Math.pow(weightStart, 3) - weightStart) * ySecondDerivatives[segmentIndex]
                    + (Math.pow(weightEnd, 3) - weightEnd) * ySecondDerivatives[segmentIndex + 1])
                    * Math.pow(segmentLength, 2) / 6.0;

            double dx = (xValues[segmentIndex + 1] - xValues[segmentIndex]) / segmentLength
                    - (3 * weightStart * weightStart - 1) * segmentLength * xSecondDerivatives[segmentIndex] / 6
                    + (3 * weightEnd * weightEnd - 1) * segmentLength * xSecondDerivatives[segmentIndex + 1] / 6;

            double dy = (yValues[segmentIndex + 1] - yValues[segmentIndex]) / segmentLength
                    - (3 * weightStart * weightStart - 1) * segmentLength * ySecondDerivatives[segmentIndex] / 6
                    + (3 * weightEnd * weightEnd - 1) * segmentLength * ySecondDerivatives[segmentIndex + 1] / 6;

            double heading = Math.atan2(dy, dx);

            path.add(new Pose2d(xPos, yPos, new Rotation2d(heading)));
        }
        return path;
    }

    //Thomas formula where it computes the second dirivatives of a triangular matric
    //First sets up the equation
    private static double[] computeSecondDerivatives(double[] time, double[] values) {
        int numPoints = time.length;
        double[] interval = new double[numPoints - 1];

        for (int i = 0; i < numPoints - 1; i++) {
            interval[i] = time[i + 1] - time[i];
        }

        double[] rhs = new double[numPoints];
        for (int i = 1; i < numPoints - 1; i++) {
            double slopeNext = (values[i + 1] - values[i]) / interval[i];
            double slopePrev = (values[i] - values[i - 1]) / interval[i - 1];
            rhs[i] = 3 * (slopeNext - slopePrev);
        }

        double[] mainDiag = new double[numPoints];
        double[] upperDiag = new double[numPoints];
        double[] lowerDiag = new double[numPoints];
        double[] secondDeriv = new double[numPoints];

        mainDiag[0] = 1;
        for (int i = 1; i < numPoints - 1; i++) {
            mainDiag[i] = 2 * (time[i + 1] - time[i - 1]);
            upperDiag[i] = interval[i];
            lowerDiag[i] = interval[i - 1];
        }
        mainDiag[numPoints - 1] = 1;

        // Forward elimination
        for (int i = 1; i < numPoints; i++) {
            double scale = lowerDiag[i] / mainDiag[i - 1];
            mainDiag[i] -= scale * upperDiag[i - 1];
            rhs[i] -= scale * rhs[i - 1];
        }

        // Back substitution
        secondDeriv[numPoints - 1] = rhs[numPoints - 1] / mainDiag[numPoints - 1];
        for (int i = numPoints - 2; i >= 0; i--) {
            secondDeriv[i] = (rhs[i] - upperDiag[i] * secondDeriv[i + 1]) / mainDiag[i];
        }

        return secondDeriv;
    }
    public void DriveSpline(Pose2d currentPose, List<Translation2d> midPoints, Pose2d targetPose){
           List<Pose2d> Points = generateCubicSplinePath(currentPose, midPoints, targetPose); //Get a returned list of points
           Log.i("MOVING ROBOT: ", "SPLINE MOVEMENT");
            for (Pose2d point : Points) {
                moveLinearAndTurn(point);
            }
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
}