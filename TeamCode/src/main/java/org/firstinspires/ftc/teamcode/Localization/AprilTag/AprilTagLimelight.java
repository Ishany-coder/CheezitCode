package org.firstinspires.ftc.teamcode.Localization.AprilTag;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.google.ar.core.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class AprilTagLimelight {
    //LIMELIGHT CONSTANTS
    public static double height = 10;
    public static double angle = 30;
    private Limelight3A limelight;
    private int AprilTagPipeline = 0; //AprilTag pipeling
    public AprilTagLimelight(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "Limelight"); // get the limelight
        limelight.pipelineSwitch(AprilTagPipeline);
        limelight.start(); // start the limelight
    }
    public Pose2d calculate(Pose2d currentPose){
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            double tx = result.getTx();
            double ty = result.getTy();
            double y = Math.tan(ty + angle) * height; // get the distance from the april tag
            double x = y/Math.tan(tx);
            return new Pose2d(currentPose.getX() + x, currentPose.getY() + y, new Rotation2d(currentPose.getHeading() + tx));
        }
        return currentPose; // no april tag detected so return current Pose
    }
}
