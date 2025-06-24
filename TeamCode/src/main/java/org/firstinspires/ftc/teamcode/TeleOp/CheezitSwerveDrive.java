package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="cheezitsSwerve", group="CheezitsTeleOp")
public class CheezitSwerveDrive extends LinearOpMode {
    private double turn;
    private double xpos;
    private double ypos;
    private double Angle;
    CoaxialDrive mySwerve;

    @Override
    public void runOpMode() throws InterruptedException {
        mySwerve = new CoaxialDrive(this.hardwareMap);

        while (opModeIsActive()) {
            // Read gamepad input
            turn = -gamepad1.left_stick_x;
            xpos = -gamepad1.right_stick_x;
            ypos = -gamepad1.right_stick_y;

            // Calculate servo position for turning
            Angle = mySwerve.getAngle(ypos, xpos);

            // Schedule turning the wheels first, then moving forward
            try {
                if (ypos > 0) {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> mySwerve.turn(Angle)), // Turn wheels
                                    new InstantCommand(() -> mySwerve.moveForward()) // Move forward
                            )
                    );
                } else if (ypos < 0) {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> mySwerve.turn(Angle)),
                                    new InstantCommand(() -> mySwerve.moveBackward())
                            )
                    );
                }
                if(turn > 0){
                    CommandScheduler.getInstance().schedule(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> mySwerve.turn(Angle)),
                                    new InstantCommand(() -> mySwerve.moveForward())
                            )
                    );
                }
                else if(turn < 0){
                    CommandScheduler.getInstance().schedule(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> mySwerve.turn(Angle)),
                                    new InstantCommand(() -> mySwerve.moveBackward())
                            )
                    );
                }
            }
            catch (Exception e){
                telemetry.addData("Error", e);
                telemetry.update();
            }
        }
    }
}