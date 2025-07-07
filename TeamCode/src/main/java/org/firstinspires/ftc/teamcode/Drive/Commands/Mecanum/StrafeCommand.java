package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

public class StrafeCommand extends InstantCommand {

    public StrafeCommand(MecanumDriveSubsystem drive, boolean right) {
        super(() -> {
            double strafePower = right ? 1.0 : -1.0;
            drive.drive(0, strafePower, 0);
        });
    }
}