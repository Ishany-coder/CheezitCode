package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;
import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

import java.util.List;

public class SplineCommand extends CommandBase {
    private List<Pose2d> path;
    private CoaxialDrive drive;
    private double vel, accel, decel;
    public SplineCommand(List<Pose2d> path, double vel, double accel, double decel){
        this.accel = accel;
        this.vel = vel;
        this.decel = decel;
        this.path = path;
    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        drive.MoveSplineWithSplineHeading(path, vel, accel, decel);
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
