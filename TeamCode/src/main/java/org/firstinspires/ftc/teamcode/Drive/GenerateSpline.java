package org.firstinspires.ftc.teamcode.Drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayList;
import java.util.List;

//change the code so at certain points it can use 1 motor to turn and move the wheel simultaneously
@Config
public class GenerateSpline {
    public GenerateSpline(){
        Log.i("GENERATE SPLINE: ", "INITIALIZED");
    }

    public double maxVelocitySpline = RobotConstants.maxVelocitySpline;
    public double maxAccelSpline = RobotConstants.maxAccelSpline;
    public double robotRadius = RobotConstants.robotRadius;

    //function to move robot linearly to a point

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
    }

    public static List<Pose2d> makeCubicSpline(Pose2d startPose, List<Translation2d> interiorPoints, Pose2d endPose) {
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

    public List<Pose2d> makeParabolicSpline(Pose2d currentPose, double obstacleRadius, Pose2d obstaclePos, double addedGapfromRobotToobs){
        List<Pose2d> path = new ArrayList<>();
        double lineLength = obstacleRadius + addedGapfromRobotToobs + robotRadius;
        double SlopeOfLine = -(currentPose.getX() - obstaclePos.getX())/(currentPose.getY() - obstaclePos.getY());
        double angle = Math.atan(SlopeOfLine);
        double newX = Math.cos(angle) * lineLength;
        double newY = Math.sin(angle) * lineLength;
        Translation2d newPose = new Translation2d(newX, newY);
        double slopeOfParab = (currentPose.getY() - newPose.getY()) / Math.pow((currentPose.getX() - newPose.getX()), 2);
        for(double i = currentPose.getX(); i < newPose.getX(); i += 0.1){
            double dx = i - newPose.getX();
            double YofParab = slopeOfParab * Math.pow((currentPose.getX() - newPose.getX()), 2) + newPose.getY();
            double dy = YofParab - newPose.getY();
            double Heading = Math.toDegrees(Math.atan(dy/dx));
            path.add(new Pose2d(i, YofParab, new Rotation2d(Heading)));
        }
        return path;
    }
    public static List<Pose2d> makeBezierWithHeading(
            Pose2d startPose,
            Pose2d endPose,
            double startControlDistance,
            double endControlDistance,
            int numPoints
    ) {
        Vector2d P0 = new Vector2d(startPose.getX(), startPose.getY());
        Vector2d P3 = new Vector2d(endPose.getX(), endPose.getY());

        Vector2d control1 = new Vector2d(
                startControlDistance * Math.cos(Math.toRadians(startPose.getHeading())),
                startControlDistance * Math.sin(Math.toRadians(startPose.getHeading()))
        );

        Vector2d control2 = new Vector2d(
                endControlDistance * Math.cos(Math.toRadians(endPose.getHeading())),
                endControlDistance * Math.sin(Math.toRadians(endPose.getHeading()))
        );

        Vector2d P1 = P0.plus(control1);
        Vector2d P2 = P3.minus(control2);

        List<Pose2d> curvePoses = new ArrayList<>();

        for (int i = 0; i <= numPoints; i++) {
            double t = (double) i / numPoints;
            double oneMinusT = 1 - t;

            // Curve position
            double x = Math.pow(oneMinusT, 3) * P0.getX()
                    + 3 * Math.pow(oneMinusT, 2) * t * P1.getX()
                    + 3 * oneMinusT * Math.pow(t, 2) * P2.getX()
                    + Math.pow(t, 3) * P3.getX();

            double y = Math.pow(oneMinusT, 3) * P0.getY()
                    + 3 * Math.pow(oneMinusT, 2) * t * P1.getY()
                    + 3 * oneMinusT * Math.pow(t, 2) * P2.getY()
                    + Math.pow(t, 3) * P3.getY();

            // Derivative (tangent) for heading
            double dx = 3 * Math.pow(oneMinusT, 2) * (P1.getX() - P0.getX())
                    + 6 * oneMinusT * t * (P2.getX() - P1.getX())
                    + 3 * Math.pow(t, 2) * (P3.getX() - P2.getX());

            double dy = 3 * Math.pow(oneMinusT, 2) * (P1.getY() - P0.getY())
                    + 6 * oneMinusT * t * (P2.getY() - P1.getY())
                    + 3 * Math.pow(t, 2) * (P3.getY() - P2.getY());

            double headingRadians = Math.atan2(dy, dx);
            double headingDegrees = Math.toDegrees(headingRadians);

            curvePoses.add(new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees)));
        }
        return curvePoses;
    }
    public List<Pose2d> makeBezierWithHeadingV2(
            Pose2d startPose,
            Pose2d endPose,
            Translation2d controlPoint1,
            Translation2d controlPoint2
    ) {
        int numPoints = 20; //get 20 points
        Log.i("MOVING ROBOT: ", "BEZIER MOVEMENT");
        Vector2d P0 = new Vector2d(startPose.getX(), startPose.getY());
        Vector2d P3 = new Vector2d(endPose.getX(), endPose.getY());

        Vector2d P1 = new Vector2d(controlPoint1.getX(), controlPoint1.getY());

        Vector2d P2 = new Vector2d(controlPoint2.getX(), controlPoint2.getY());

        List<Pose2d> curvePoses = new ArrayList<>();
        Log.i("INITIALIZED: ", "CONTROL POINTS");
        for (int i = 0; i <= numPoints; i++) {
            double t = (double) i / numPoints;
            double oneMinusT = 1 - t;
            Log.i("GOT T value: ", "T: " + t);
            // Curve position
            double x = Math.pow(oneMinusT, 3) * P0.getX()
                    + 3 * Math.pow(oneMinusT, 2) * t * P1.getX()
                    + 3 * oneMinusT * Math.pow(t, 2) * P2.getX()
                    + Math.pow(t, 3) * P3.getX();
            Log.i("GOT X value: ", "X: " + x);
            double y = Math.pow(oneMinusT, 3) * P0.getY()
                    + 3 * Math.pow(oneMinusT, 2) * t * P1.getY()
                    + 3 * oneMinusT * Math.pow(t, 2) * P2.getY()
                    + Math.pow(t, 3) * P3.getY();
            Log.i("GOT Y value: ", "Y: " + y);
            // Derivative (tangent) for heading
            double dx = 3 * Math.pow(oneMinusT, 2) * (P1.getX() - P0.getX())
                    + 6 * oneMinusT * t * (P2.getX() - P1.getX())
                    + 3 * Math.pow(t, 2) * (P3.getX() - P2.getX());
            Log.i("GOT DX value: ", "DX: " + dx);
            double dy = 3 * Math.pow(oneMinusT, 2) * (P1.getY() - P0.getY())
                    + 6 * oneMinusT * t * (P2.getY() - P1.getY())
                    + 3 * Math.pow(t, 2) * (P3.getY() - P2.getY());
            Log.i("GOT DY value: ", "DY: " + dy);
            double headingRadians = Math.atan2(dy, dx);
            double headingDegrees = Math.toDegrees(headingRadians);

            curvePoses.add(new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees)));
            Log.i("ADDED NEW POSE: ", "X: " + x + " Y: " + y + " Heading: " + headingDegrees + " degrees");
        }
        return curvePoses;
    }

    public static List<Pose2d> generateHermiteCurve(
            Pose2d start, double startMag,
            Pose2d end, double endMag) {
        int numPoints = 100; // Find 100 points along curve
        List<Pose2d> path = new ArrayList<>();

        // Tangent vectors
        double m0x = Math.cos(Math.toRadians(start.getHeading())) * startMag;
        double m0y = Math.sin(Math.toRadians(start.getHeading())) * startMag;
        double m1x = Math.cos(Math.toRadians(end.getHeading())) * endMag;
        double m1y = Math.sin(Math.toRadians(end.getHeading())) * endMag;

        for (int i = 0; i <= numPoints; i++) {
            double t = (double) i / numPoints; // value from 0 to 1 along curve

            double h00 = 2 * t * t * t - 3 * t * t + 1; // How much start to keep 0 full 1 none
            double h10 = t * t * t - 2 * t * t + t; // start tangent slope grows and fades back at 1
            double h01 = -2 * t * t * t + 3 * t * t; // How much end to keep 0 none 1 full
            double h11 = t * t * t - t * t; // end tangent slope 0 and 1 is none peaks mid

            // h00 * start.getX() how much start point x to use
            // h10 * m0x how much start tangents x influences curve
            // h01 * end.getX() how much end point x to use
            // h11 * m1x how much end tangents x influences curve
            double x = h00 * start.getX() + h10 * m0x + h01 * end.getX() + h11 * m1x;
            double y = h00 * start.getY() + h10 * m0y + h01 * end.getY() + h11 * m1y;

            // Derivative for heading
            double dx = (6 * t * t - 6 * t) * start.getX() + (3 * t * t - 4 * t + 1) * m0x
                    + (-6 * t * t + 6 * t) * end.getX() + (3 * t * t - 2 * t) * m1x;
            double dy = (6 * t * t - 6 * t) * start.getY() + (3 * t * t - 4 * t + 1) * m0y
                    + (-6 * t * t + 6 * t) * end.getY() + (3 * t * t - 2 * t) * m1y;

            double heading = Math.atan2(dy, dx);

            path.add(new Pose2d(x, y, new Rotation2d(heading)));
        }

        return path;
    }
    public static List<Pose2d> generateQuinticSpline(
            Pose2d startPos, // x y pos
            Translation2d startVel, // x and y vel
            Translation2d startAccel, // x and y accel
            Pose2d endPos, // x y pos
            Translation2d endVel, // x and y vel
            Translation2d endAccel // x and y accel
    ) {
        int numPoints = 100; //Generate 100 points
        List<Pose2d> spline = new ArrayList<>();

        for (int i = 0; i <= numPoints; i++) {
            double t = (double) i / numPoints;

            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t3 * t;
            double t5 = t4 * t;

            // Quintic basis functions
            double b0 = 1 - 10 * t3 + 15 * t4 - 6 * t5;
            double b1 = t - 6 * t3 + 8 * t4 - 3 * t5;
            double b2 = 0.5 * t2 - 1.5 * t3 + 1.5 * t4 - 0.5 * t5;
            double b3 = 10 * t3 - 15 * t4 + 6 * t5;
            double b4 = -4 * t3 + 7 * t4 - 3 * t5;
            double b5 = 0.5 * t3 - t4 + 0.5 * t5;

            // Position
            double x = b0 * startPos.getX() + b1 * startVel.getX() + b2 * startAccel.getX()
                    + b3 * endPos.getX() + b4 * endVel.getX() + b5 * endAccel.getX();

            double y = b0 * startPos.getY() + b1 * startVel.getY() + b2 * startAccel.getY()
                    + b3 * endPos.getY() + b4 * endVel.getY() + b5 * endAccel.getY();

            // Estimate heading from dx/dy of blended vectors
            double dx = b1 * startVel.getX() + b2 * startAccel.getX()
                    + b4 * endVel.getX() + b5 * endAccel.getX();

            double dy = b1 * startVel.getY() + b2 * startAccel.getY()
                    + b4 * endVel.getY() + b5 * endAccel.getY();

            double heading = Math.atan2(dy, dx);
            spline.add(new Pose2d(x, y, new Rotation2d(heading)));
        }

        return spline;
    }
}