package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

import java.util.List;

public class DriveCommand extends CommandBase {
    private double x,y,t;
    public DriveCommand(double x, double y, double t){
        this.x = x;
        this.y = y;
        this.t = t;
    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        MecanumDriveSubsystem.drive(y,x,t);
    }
    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    @Override
    public void end(boolean Interupted){
        MecanumDriveSubsystem.stop();
    }
}
