package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;

import com.arcrobotics.ftclib.command.InstantCommand;

public class TurnCommand extends InstantCommand {
    public TurnCommand(CoaxialDrive drive, double angle) {
        super(() -> drive.turn(angle));
    }
}
