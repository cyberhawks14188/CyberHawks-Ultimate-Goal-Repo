package org.firstinspires.ftc.teamcode.TestPrograms;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.ControlHub.JustMotorRobotHardware;

@TeleOp
public class ColorSensor extends LinearOpMode {
    JustMotorRobotHardware robot = new JustMotorRobotHardware();

    public void runOpMode() {
        robot.init(hardwareMap);
        final float[] hsvValues = new float[3];
        if (robot.ColorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight)robot.ColorSensor1).enableLight(true);
        }
        waitForStart();
        while (opModeIsActive()) {
            robot.ColorSensor1.setGain(10);
            NormalizedRGBA colors = robot.ColorSensor1.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.update();
        }
    }
}
