package org.firstinspires.ftc.teamcode.TeleOpCode.BallDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ComplexBallDrive", group = "BallDrive")
public class ComplexBallDrive extends LinearOpMode {

    BallDriveHardware robot = new BallDriveHardware();

    static double robotA = -13;
    static double robotB = 6.5;

    static double robotTurningMax = Math.max(robotA,robotB);
    static double robotTurningX = robotA/robotTurningMax;
    static double robotTurningY = robotB/robotTurningMax;
    double inputY = 0;
    double inputX = 0;
    double inputZ = 0;
    double leftBallTurningX, leftBallTurningY, rightBallTurningX, rightBallTurningY;
    double leftBallXRaw = 0, leftBallYRaw = 0, rightBallXRaw = 0, rightBallYRaw = 0;
    double rawMax;
    double leftBallX, leftBallY, rightBallX, rightBallY;
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
    while ((opModeIsActive())) {

        inputY = gamepad1.left_stick_x;
        inputX = gamepad1.left_stick_y;
        inputZ = gamepad1.right_stick_x;

        leftBallTurningX = inputZ * robotTurningX;
        leftBallTurningY = inputZ * -robotTurningY;
        rightBallTurningX = inputZ * -robotTurningX;
        rightBallTurningY = inputZ * -robotTurningY;

        leftBallXRaw = inputX + leftBallTurningX;
        leftBallYRaw = inputY + leftBallTurningY;
        rightBallXRaw = inputX + rightBallTurningX;
        rightBallYRaw = inputY + rightBallTurningY;

        rawMax = Math.max(Math.max(Math.abs(leftBallXRaw), Math.abs(leftBallYRaw)), Math.max(Math.abs(rightBallXRaw), Math.abs(rightBallYRaw)));

        if (rawMax > 1 && rawMax != 0) {
            leftBallX = leftBallXRaw / rawMax;
            leftBallY = leftBallYRaw / rawMax;
            rightBallX = rightBallXRaw / rawMax;
            rightBallY = rightBallYRaw / rawMax;
        }else if(rawMax < -1 && rawMax != 0){
            leftBallX = leftBallXRaw / rawMax;
            leftBallY = leftBallYRaw / rawMax;
            rightBallX = rightBallXRaw / rawMax;
            rightBallY = rightBallYRaw / rawMax;
        }else{
            leftBallX = leftBallXRaw;
            leftBallY = leftBallYRaw;
            rightBallX = rightBallXRaw;
            rightBallY = rightBallYRaw;
        }
        telemetry.addData("leftBallX", leftBallX);
        telemetry.addData("leftBallY", leftBallY);
        telemetry.addData("rightBallX", rightBallX);
        telemetry.addData("rightBallY", rightBallY);
        telemetry.addData("RAWleftBallX", leftBallXRaw);
        telemetry.addData("RAWleftBallY", leftBallYRaw);
        telemetry.addData("RAWrightBallX", rightBallXRaw);
        telemetry.addData("RAWrightBallY", rightBallYRaw);
        telemetry.addData("rawmax", rawMax);
        telemetry.update();

        robot.LB_X_M.setPower(leftBallX);
        robot.LB_Y_M.setPower(leftBallY);
        robot.RB_X_M.setPower(rightBallX);
        robot.RB_Y_M.setPower(rightBallY);
    }
    }
}