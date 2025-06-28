package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.RobotHardware;


@TeleOp(name="cheezitsSwerve", group="CheezitsTeleOp")
public class CheezitSwerveDrive extends LinearOpMode {
    private double turn;
    private double xpos;
    private double ypos;
    private double Angle;
    RobotHardware myHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);

        while (opModeIsActive()) {
            // Read gamepad input
            turn = -gamepad1.left_stick_x;
            xpos = -gamepad1.right_stick_x;
            ypos = -gamepad1.right_stick_y;

            // Calculate servo position for turning
            Angle = myHardware.getAngle(ypos, xpos);

            // Schedule turning the wheels first, then moving forward
            try {
                if (ypos > 0) {
                    Log.i("CHEEZITS MOVING ROBOT: ", " FORWARD SERVO ANGLE: " + Angle);
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> myHardware.turn(Angle)), // Turn wheels
                                    new InstantCommand(() -> myHardware.moveForward()) // Move forward
                            )
                    );
                } else if (ypos < 0) {
                    Log.i("CHEEZITS MOVING ROBOT: ", "BACKWARD SERVO ANGLE: " + Angle);
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> myHardware.turn(Angle)),
                                    new InstantCommand(() -> myHardware.moveBackward())
                            )
                    );
                }
                // Logic to turn bobot
                if(turn > 0){
                    Log.i("CHEEZITS MOVING ROBOT: ", "TURNING FORWARD SERVO ANGLE: " + Angle);
                    CommandScheduler.getInstance().schedule(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> myHardware.turn(Angle)),
                                    new InstantCommand(() -> myHardware.moveForward())
                            )
                    );
                }
                else if(turn < 0){
                    Log.i("CHEEZITS MOVING ROBOT: ", "TURNING BACKWARD SERVO ANGLE: " + Angle);
                    CommandScheduler.getInstance().schedule(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> myHardware.turn(Angle)),
                                    new InstantCommand(() -> myHardware.moveBackward())
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