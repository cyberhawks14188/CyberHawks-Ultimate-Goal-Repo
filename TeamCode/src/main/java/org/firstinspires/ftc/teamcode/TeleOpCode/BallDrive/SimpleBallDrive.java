package org.firstinspires.ftc.teamcode.TeleOpCode.BallDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SimpleBallDrive", group = "BallDrive")
public class SimpleBallDrive extends LinearOpMode {

    BallDriveHardware robot = new BallDriveHardware();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
    while(opModeIsActive()){
        //sets Y direction motors
        robot.RB_Y_M.setPower(gamepad1.left_stick_x);
        robot.LB_Y_M.setPower(gamepad1.left_stick_x);

        //sets X motor power with turning included
        robot.RB_X_M.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        robot.LB_X_M.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
    }

    }
}
