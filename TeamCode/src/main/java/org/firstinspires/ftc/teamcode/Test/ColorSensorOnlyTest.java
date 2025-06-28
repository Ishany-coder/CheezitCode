package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Only Test", group = "Linear OpMode")
public class ColorSensorOnlyTest extends LinearOpMode {

    public ColorRangeSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "ColorSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance (cm)", colorSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Color", detectColor());
            telemetry.update();
        }
    }

    private String detectColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (blue > red && blue > green) {
            return "BLUE";
        } else if (red > green && red > blue) {
            return "RED";
        } else {
            return "YELLOW";
        }
    }
}