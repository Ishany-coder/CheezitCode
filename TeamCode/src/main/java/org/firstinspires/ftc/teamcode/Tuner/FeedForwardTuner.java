package org.firstinspires.ftc.teamcode.Tuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Localization.Odo.Odometery;

@Autonomous(name = "FeedForwardTuner")
public class FeedForwardTuner extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private static final double POWER = 0.6;
    private static final double DISTANCE_INCHES = 36;
    private static final double TICKS_PER_INCH = 537.7 / (Math.PI * 4); // 312 RPM GoBILDA
    private static final int SAMPLES = 200;
    private Odometery odo;

    @Override
    public void runOpMode() {
        drive = new MecanumDriveSubsystem(hardwareMap);
        odo = new Odometery(this.hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("🚨 FeedForward Tuner Ready");
        telemetry.addLine("🚗 Robot will move forward & backward");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Forward
        while(opModeIsActive()) {
            collectDataAndLog(POWER, DISTANCE_INCHES, SAMPLES, dashboard);

            // Backward
            collectDataAndLog(-POWER, DISTANCE_INCHES, SAMPLES, dashboard);
        }
        telemetry.addLine("✅ Feedforward tuning complete.");
        telemetry.update();
    }

    private void collectDataAndLog(double power, double inches, int samples, FtcDashboard dashboard) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        drive.drive(power, 0, 0);
        long prevTime = System.nanoTime();
        odo.setCurrentPose(new Pose2d(0,0,new Rotation2d(0)));
        Pose2d prevPose = odo.getCurrentPose();
        for (int i = 0; i < samples && opModeIsActive(); i++) {
            long now = System.nanoTime();
            double dt = (now - prevTime) / 1e9;
            prevTime = now;

            Pose2d currentPose = odo.getCurrentPose();
            double dx = currentPose.getX() - prevPose.getX();
            double velocity = dx / dt; // inches/sec
            double acceleration = (velocity - ((prevPose.getX() - prevPose.getX()) / dt)) / dt;

            double predictedPower = MecanumDriveSubsystem.KV * velocity + MecanumDriveSubsystem.KA * acceleration;
            double error = predictedPower - power;

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("velocity", velocity);
            packet.put("acceleration", acceleration);
            packet.put("power", power);
            packet.put("predictedPower", predictedPower);
            packet.put("error", error);
            packet.put("KV", MecanumDriveSubsystem.KV);
            packet.put("KA", MecanumDriveSubsystem.KA);

            dashboard.sendTelemetryPacket(packet);
            prevPose = currentPose;
            sleep(10); // Optional delay
        }

        drive.stop();
        sleep(500);
    }
}