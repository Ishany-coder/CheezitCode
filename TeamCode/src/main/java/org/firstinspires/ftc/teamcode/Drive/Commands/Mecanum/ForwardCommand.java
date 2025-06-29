package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;

public class ForwardCommand extends InstantCommand {
    public ForwardCommand(MecanumDriveSubsystem drive) {
        super(() -> drive.moveForward());
    }
}
