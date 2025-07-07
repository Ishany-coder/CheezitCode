package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;

import java.util.List;

public class SplineCommand extends InstantCommand {
    //Command to move in a spline
    public SplineCommand(CoaxialDrive drive, List<Pose2d> path) {
        super(() -> drive.MoveSpline(path));
    }
}
