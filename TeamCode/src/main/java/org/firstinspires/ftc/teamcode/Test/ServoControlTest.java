package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Config
@TeleOp(name="ServoTest", group="Linear OpMode")
public class ServoControlTest extends LinearOpMode {

    // Declare OpMode members.
    public static double ServoPosition = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo Servo;
    public static boolean Forward = true;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Servo = hardwareMap.get(Servo.class, "Servo");
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if(Forward){
                Servo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
                Servo.setPosition(ServoPosition);
            }
            else{
                Servo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
                Servo.setPosition(ServoPosition);
            }
        }
    }
}