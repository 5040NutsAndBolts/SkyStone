package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="color sensor",group="Auto")
public class ColorSensorTest extends OpMode
{

    public ColorSensor blockDetector = null;

    @Override
    public void init() {
        blockDetector = hardwareMap.get(ColorSensor.class, "blockDetector");
    }

    @Override
    public void loop() {

        telemetry.addData("ARGB", blockDetector.argb());
        telemetry.addData("Alpha", blockDetector.alpha());
        telemetry.addData("Red", blockDetector.red());
        telemetry.addData("Green", blockDetector.green());
        telemetry.addData("Blue", blockDetector.blue());
        telemetry.addData("Block?", blockDetector.alpha() > 200);
        telemetry.update();

    }
}
