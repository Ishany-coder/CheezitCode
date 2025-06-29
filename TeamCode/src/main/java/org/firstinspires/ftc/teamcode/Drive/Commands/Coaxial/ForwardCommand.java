package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;

import com.arcrobotics.ftclib.command.InstantCommand;

public class ForwardCommand extends InstantCommand {
    public ForwardCommand(CoaxialDrive drive) {
        super(() -> drive.moveForward());
    }
}
