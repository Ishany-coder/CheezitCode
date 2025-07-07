package org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

public class DriveCommand extends InstantCommand {
    public DriveCommand(MecanumDriveSubsystem drive, double x, double y, double angle) {
        super(() -> drive.drive(y, x, angle), drive);
    }
}
