package org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial;
import com.arcrobotics.ftclib.command.InstantCommand;

public class StrafeCommand extends InstantCommand {
    public StrafeCommand(CoaxialDrive drive, boolean right) {
        super(() -> drive.Strafe(right), drive);
    }
}