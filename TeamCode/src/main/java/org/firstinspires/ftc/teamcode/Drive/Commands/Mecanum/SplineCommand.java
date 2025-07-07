package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

import java.util.List;

public class SplineCommand extends InstantCommand {
    public SplineCommand(MecanumDriveSubsystem drive, List<Pose2d> path){
        super(() -> drive.followPath(path));
    }
}
