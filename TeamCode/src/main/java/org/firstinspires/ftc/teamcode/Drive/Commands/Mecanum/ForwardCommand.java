package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

public class ForwardCommand extends InstantCommand {
    public ForwardCommand(MecanumDriveSubsystem drive) {
        super(() -> drive.moveForward());
    }
}
