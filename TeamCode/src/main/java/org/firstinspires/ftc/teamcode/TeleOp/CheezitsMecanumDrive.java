package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Drive.Commands.Mecanum.DriveCommand;
import org.firstinspires.ftc.teamcode.Drive.MecanumDriveSubsystem;

public class CheezitsMecanumDrive extends LinearOpMode {
    public Gamepad gamepad1; // only take inputs from gamepad1 for now
    public double Forward;
    public double Strafe;
    public double Turn;
    public MecanumDriveSubsystem drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveSubsystem(this.hardwareMap);
        gamepad1 = new Gamepad();
        waitForStart();
        while (opModeIsActive()) {
            Forward = -gamepad1.left_stick_y;
            Strafe = gamepad1.left_stick_x;
            Turn = gamepad1.right_stick_x;
            CommandScheduler.getInstance().schedule(new DriveCommand(Strafe, Forward, Turn));
        }
    }
}
