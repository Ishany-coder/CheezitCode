package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;

public class BackwardsCommand extends InstantCommand {
    public BackwardsCommand(MecanumDriveSubsystem drive) {
        super(() -> drive.moveBackward());
    }
}
