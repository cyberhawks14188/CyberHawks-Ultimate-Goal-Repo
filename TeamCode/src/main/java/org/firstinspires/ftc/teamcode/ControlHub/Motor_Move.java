package org.firstinspires.ftc.teamcode.ControlHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class Motor_Move extends LinearOpMode {
    JustMotorRobotHardware robot = new JustMotorRobotHardware();

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.Motor1.setPower(-1);
            robot.Motor2.setPower(1);
            robot.Motor3.setPower(1);
            robot.Motor4.setPower(1);
        }
    }
}

