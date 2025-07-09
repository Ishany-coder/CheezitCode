package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

import java.util.List;

public class DriveToPoseCommand extends CommandBase {
    private MecanumDriveSubsystem drive;
    private Pose2d targetPose, currPose, vel;
    public DriveToPoseCommand(Pose2d targetPose, Pose2d currPose, Pose2d vel){
        this.targetPose = targetPose;
        this.currPose = currPose;
        this.vel = vel;
    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        drive.driveToPose(targetPose, currPose, vel);
    }
    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    @Override
    public void end(boolean Interupted){
        drive.stop();
    }
}
