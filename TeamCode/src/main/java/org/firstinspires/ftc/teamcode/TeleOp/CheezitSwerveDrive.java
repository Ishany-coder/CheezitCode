package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;
import org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial.DriveCommand;


@TeleOp(name="cheezitsSwerve", group="CheezitsTeleOp")
public class CheezitSwerveDrive extends LinearOpMode {
    private double turn;
    private double xpos;
    private double ypos;
    private double Angle;
    CoaxialDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new CoaxialDrive(this.hardwareMap);

        while (opModeIsActive()) {
            // Read gamepad input
            turn = -gamepad1.left_stick_x;
            xpos = -gamepad1.right_stick_x;
            ypos = -gamepad1.right_stick_y;
            // Schedule turning the wheels first, then moving forward
            CommandScheduler.getInstance().schedule(new DriveCommand(xpos, ypos, turn));
        }
    }
}