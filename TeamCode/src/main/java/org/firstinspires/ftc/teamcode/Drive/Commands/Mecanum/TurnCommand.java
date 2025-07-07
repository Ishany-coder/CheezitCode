package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

public class TurnCommand extends InstantCommand {
    public TurnCommand(MecanumDriveSubsystem drive, double angle) {
        super(() -> drive.drive(0, 0, angle));
    }
}
