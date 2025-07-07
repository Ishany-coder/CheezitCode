package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;

public class BackwardCommand extends InstantCommand {
    public BackwardCommand(CoaxialDrive drive) {
        super(() -> drive.moveBackward());
    }
}
