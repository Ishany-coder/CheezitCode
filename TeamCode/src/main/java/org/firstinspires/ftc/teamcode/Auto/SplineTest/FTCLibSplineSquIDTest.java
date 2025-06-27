package org.firstinspires.ftc.teamcode.Auto.SplineTest;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.Squid.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.TeleOp.CoaxialDrive;

@Autonomous(name = "FTCLib Spline SquID Drive", group = "Cheezits")
public class FTCLibSplineSquIDTest extends CommandOpMode {

    private CoaxialDrive drive;
    private DrivetrainSquIDController squidController;
    private Pose2d currentPose;
    private Pose2d targetPose;

    @Override
    public void initialize() {
        // Initialize drive system and SquID controller
        drive = new CoaxialDrive(hardwareMap);
        squidController = new DrivetrainSquIDController();

        // Set initial position (robot starts at 0,0 facing 0°)
        currentPose = new Pose2d(0, 0, new Rotation2d(0));

        // Set target position (robot moves to 24,24 and rotates 90 degrees)
        targetPose = new Pose2d(24, 24, new Rotation2d(Math.toRadians(90)));

        // Schedule movement sequence using FTC Lib Commands
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new DriveSplineCommand(drive, squidController, currentPose, targetPose, 2.0), // Move smoothly to target position
                        new InstantCommand(() -> drive.stop()) // Stop servos when done
                )
        );
    }
}