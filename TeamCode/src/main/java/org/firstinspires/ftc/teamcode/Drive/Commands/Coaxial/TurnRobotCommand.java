package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;

public class TurnRobotCommand extends ParallelCommandGroup {
    public TurnRobotCommand(CoaxialDrive drive, double angle, boolean Right) {
        new ParallelCommandGroup(
                new TurnCommand(drive, angle),
                new ForwardCommand(drive)
        );
    }
}
