package org.firstinspires.ftc.teamcode.Drive.Spline;

import static org.firstinspires.ftc.teamcode.Drive.CoaxialDrive.KA;
import static org.firstinspires.ftc.teamcode.Drive.CoaxialDrive.KV;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.Controller.FeedForward.FeedforwardController;
import org.firstinspires.ftc.teamcode.Controller.PurePursuit.PurePursuitController;
import org.firstinspires.ftc.teamcode.Controller.squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Controller.squid.SquIDController;
import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;
import org.firstinspires.ftc.teamcode.Localization.Odometery;

import java.util.List;

@Config
public class MainSpline {
    public Odometery odo;
    public DrivetrainSquIDController squid;
    public CoaxialDrive drive;
    public MainSpline(Odometery odo, DrivetrainSquIDController squid, CoaxialDrive drive){
        this.odo = odo;
        this.squid = squid;
        this.drive = drive;
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
                drive.DriveToPose(correction, currentPose, velocityVector);
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
            drive.DriveToPose(new Pose2d(
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
}
