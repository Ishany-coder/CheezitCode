package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;

public class ForwardCommand extends InstantCommand {
    public ForwardCommand(CoaxialDrive drive) {
        super(() -> drive.moveForward());
    }
}
