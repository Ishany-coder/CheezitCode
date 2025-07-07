package org.firstinspires.ftc.teamcode.Auto.Tuner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controller.PID.PID;

@Config
@TeleOp(name = "PID Test")
public class PIDTuner extends LinearOpMode {
    public double kP = 0.1;
    public double kI = 0.01;
    public double kD = 0.05;
    @Override
    public void runOpMode() {
        PID pid = new PID(kP, kI, kD); // Tune as needed
        pid.setOutputLimits(-1, 1);
        pid.setTolerance(0.05); // Acceptable error

        double setpoint = 100.0;
        double current = 0.0;

        telemetry.addLine("Ready to test PID");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double power = pid.calculate(setpoint, current);
            current += power; // Simulate applying power (mock system response)

            telemetry.addData("Setpoint", setpoint);
            telemetry.addData("Current", current);
            telemetry.addData("Error", setpoint - current);
            telemetry.addData("Power", power);
            telemetry.addData("At Setpoint", pid.atSetpoint(setpoint, current));
            telemetry.update();

            if (pid.atSetpoint(setpoint, current)) {
                telemetry.addLine("Reached setpoint!");
                telemetry.update();
                break;
            }

            sleep(50); // simulate control loop timing
        }
    }
}