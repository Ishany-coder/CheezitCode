package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial.BackwardCommand;
import org.firstinspires.ftc.teamcode.Drive.CoaxialDrive;
import org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial.ForwardCommand;
import org.firstinspires.ftc.teamcode.Drive.Commands.Coaxial.TurnCommand;


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

            // Calculate servo position for turning
            Angle = drive.getAngle(ypos, xpos);

            // Schedule turning the wheels first, then moving forward
            try {
                if (ypos > 0) {
                    Log.i("CHEEZITS MOVING ROBOT: ", " FORWARD SERVO ANGLE: " + Angle);
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new TurnCommand(drive, Angle), // Turn wheels
                                    new ForwardCommand(drive)// Move forward
                            )
                    );
                } else if (ypos < 0) {
                    Log.i("CHEEZITS MOVING ROBOT: ", "BACKWARD SERVO ANGLE: " + Angle);
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new TurnCommand(drive, Angle),
                                    new BackwardCommand(drive)
                            )
                    );
                }
                // Logic to turn bobot
                if(turn > 0){
                    Log.i("CHEEZITS MOVING ROBOT: ", "TURNING FORWARD SERVO ANGLE: " + Angle);
                    CommandScheduler.getInstance().schedule(
                            new ParallelCommandGroup(
                                    new TurnCommand(drive, Angle),
                                    new ForwardCommand(drive)
                            )
                    );
                }
                else if(turn < 0){
                    Log.i("CHEEZITS MOVING ROBOT: ", "TURNING BACKWARD SERVO ANGLE: " + Angle);
                    CommandScheduler.getInstance().schedule(
                            new ParallelCommandGroup(
                                    new TurnCommand(drive, Angle),
                                    new BackwardCommand(drive)
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