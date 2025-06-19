package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.SwerveDrive;

@TeleOp(name="cheezitsSwerve", group="CheezitsTeleOp")
public class CheezitSwerveDrive extends LinearOpMode {
    private double turn;
    private double xpos;
    private double ypos;
    private double ServoPosition;
    SwerveDrive myHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new SwerveDrive(this.hardwareMap);

        while (opModeIsActive()) {
            // Read gamepad input
            turn = -gamepad1.left_stick_x;
            xpos = -gamepad1.right_stick_x;
            ypos = -gamepad1.right_stick_y;

            // Calculate servo position for turning
            ServoPosition = myHardware.getAngle(ypos, xpos);

            // Schedule turning the wheels first, then moving forward
            //if ypos > 0 move forward
            if(ypos > 0) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> myHardware.turnDriveMotors(ServoPosition)), // Turn wheels
                                new InstantCommand(myHardware::moveForward) // Move forward
                        )
                );
                // at any point if the ypos is less then 0 then move back
            } else if (ypos < 0) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> myHardware.turnDriveMotors(ServoPosition)),
                                new InstantCommand(() -> myHardware.moveBackward())
                        )
                );

            }
            // as long as turn doesnt equal null we turn
            if (turn != 0) {
                myHardware.turn(turn);
            }
        }
    }
}