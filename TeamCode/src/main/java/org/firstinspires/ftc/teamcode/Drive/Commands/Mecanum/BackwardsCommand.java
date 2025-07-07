package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

public class BackwardsCommand extends InstantCommand {
    public BackwardsCommand(MecanumDriveSubsystem drive) {
        super(() -> drive.moveBackward());
    }
}
