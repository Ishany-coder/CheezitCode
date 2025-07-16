package org.firstinspires.ftc.teamcode.Localization.Odo;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Up is x and right is y
// like so 🔼x ➡️y 🔄 Heading
@Config
public class Odometery {
    public DcMotorEx Odo1;
    public DcMotorEx Odo2;
    public DcMotorEx Odo3;
    private Pose2d currentPose;

    public double Encoder1PrevTicks = Double.NaN;
    public double Encoder2PrevTicks = Double.NaN;
    public double Encoder3PrevTicks = Double.NaN;

    private BNO055IMU imu;
    private double lastIMUHeadingDeg = 0;
    public static double IMUweight = 0.8;
    public static double Encweight = 0.2;

    public Odometery(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);
        while (!imu.isGyroCalibrated()) {}
        this.Odo1 = hardwareMap.get(DcMotorEx.class, "RightFront");
        this.Odo2 = hardwareMap.get(DcMotorEx.class, "LeftFront");
        this.Odo3 = hardwareMap.get(DcMotorEx.class, "RightBack");
        lastIMUHeadingDeg = getIMUYaw();
    }

    public double getIMUYaw() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return normalizeDegrees(angles.firstAngle); // Z axis yaw
    }

    private double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    //currently uses linear odo but soon use arcs for better accuracy as it takes less samples for same result
    public Pose2d updatePose(Pose2d currentPose) {
        if (Double.isNaN(Encoder1PrevTicks)) {
            Encoder1PrevTicks = Odo1.getCurrentPosition();
            Encoder2PrevTicks = Odo2.getCurrentPosition();
            Encoder3PrevTicks = Odo3.getCurrentPosition();
            lastIMUHeadingDeg = getIMUYaw();
            return currentPose;
        }

        double DeltaEncoder1Ticks = Odo1.getCurrentPosition() - Encoder1PrevTicks;
        double DeltaEncoder2Ticks = Odo2.getCurrentPosition() - Encoder2PrevTicks;
        double DeltaEncoder3Ticks = Odo3.getCurrentPosition() - Encoder3PrevTicks;

        double I = (2 * Math.PI * WheelRadius) / EncoderTicksPerRev;
        double DeltaX = I * ((DeltaEncoder1Ticks + DeltaEncoder2Ticks) / 2);
        double DeltaThetaEnc = Math.toDegrees(I * ((DeltaEncoder1Ticks - DeltaEncoder2Ticks) / LeftAndRightEncoderDist));
        double DeltaY = I * (DeltaEncoder3Ticks - (FrontEncoderOffset * (DeltaEncoder2Ticks - DeltaEncoder1Ticks) / LeftAndRightEncoderDist));

        // Encoder-based new heading
        double EncodedTheta = normalizeDegrees(Math.toDegrees(currentPose.getHeading()) + DeltaThetaEnc);

        // IMU absolute heading
        double IMUTheta = getIMUYaw();

        // Fuse them
        double FusedTheta = normalizeDegrees(IMUweight * IMUTheta + Encweight * EncodedTheta);
        double thetaRad = Math.toRadians(FusedTheta);

        double NewX = currentPose.getX() + DeltaX * Math.cos(thetaRad) - DeltaY * Math.sin(thetaRad);
        double NewY = currentPose.getY() + DeltaX * Math.sin(thetaRad) + DeltaY * Math.cos(thetaRad);

        Encoder1PrevTicks = Odo1.getCurrentPosition();
        Encoder2PrevTicks = Odo2.getCurrentPosition();
        Encoder3PrevTicks = Odo3.getCurrentPosition();
        lastIMUHeadingDeg = IMUTheta;

        currentPose = new Pose2d(NewX, NewY, Rotation2d.fromDegrees(FusedTheta));
        return (currentPose);
    }
    public Pose2d getCurrentPose(){
        return currentPose; // return the current Pose after calculation
    }
    public void setCurrentPose(Pose2d currentPose){
        this.currentPose = currentPose;
    }
}