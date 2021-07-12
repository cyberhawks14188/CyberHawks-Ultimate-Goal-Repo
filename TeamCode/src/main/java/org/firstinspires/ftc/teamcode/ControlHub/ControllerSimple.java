package org.firstinspires.ftc.teamcode.ControlHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ControllerSimple extends LinearOpMode {
    JustMotorRobotHardware robot = new JustMotorRobotHardware();

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.Motor1.setPower(gamepad1.left_stick_y);
            robot.Motor2.setPower(gamepad1.left_stick_x);
            telemetry.addData("motor1", robot.Motor1.getPower());
        }
    }
}
