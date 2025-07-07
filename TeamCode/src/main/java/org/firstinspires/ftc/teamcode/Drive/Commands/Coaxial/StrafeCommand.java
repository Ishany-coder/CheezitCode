package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;

public class StrafeCommand extends InstantCommand {
    public StrafeCommand(CoaxialDrive drive, boolean right) {
        super(() -> drive.Strafe(right), drive);
    }
}