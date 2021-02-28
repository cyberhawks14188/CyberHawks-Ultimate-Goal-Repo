package org.firstinspires.ftc.teamcode.TeleOpCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.Odometry;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.RingSystem;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.ShooterSystem;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.WobbleGoalArm;

@TeleOp
public class MultiClassTeleop extends LinearOpMode {
    Odometry OdoClass = new Odometry();
    RingSystem RingClass = new RingSystem();
    ShooterSystem ShooterClass = new ShooterSystem();
    WobbleGoalArm WobbleArmClass = new WobbleGoalArm();
    DriveTrain DrivetrainClass = new DriveTrain();

    @Override
    public void runOpMode() {
        //Calling upon the HardwareMap and other classes
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
//assigning stating types for sensors
        robot.Ring1_CS.setGain(10);
        robot.Ring2_CS.setGain(10);
        robot.Ring3_CS.setGain(10);

        //Waits for the play button to be pressed
        waitForStart();
        //mMin loop that our TeleOp loops in
        while (opModeIsActive()) {
            NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors();
            NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors();
            NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();
            //Calling to the Classes and the methods inside of them to run the calculations and set points.
            DrivetrainClass.DriveBase(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper);
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
            RingClass.RingSystemControl(gamepad1.a, gamepad1.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad1.back);
            ShooterClass.shooterControl(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.y, gamepad1.x, gamepad1.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage());
            WobbleArmClass.WobbleControl(gamepad1.left_trigger, gamepad1.dpad_up, gamepad1.dpad_down, robot.WB_PT.getVoltage());
            //Setting Motor Power
            robot.LF_M.setPower(DrivetrainClass.LFMReturn());
            robot.LB_M.setPower(DrivetrainClass.LBMReturn());
            robot.RF_M.setPower(DrivetrainClass.RFMReturn());
            robot.RB_M.setPower(DrivetrainClass.RBMReturn());
            robot.IN_M.setPower(RingClass.intakePowerReturn());
            robot.STG_M.setPower(RingClass.stagerPowerRetun());
            robot.STOP_S.setPosition(RingClass.stopperSetReturn());
            robot.SOT_M.setPower(ShooterClass.shooterMotorPowerReturn());
            robot.SOT_S.setPower(ShooterClass.sotAnglePowerReturn());
            robot.WB_M.setPower(WobbleArmClass.wobblePowerReturn());
            robot.GRIP_S.setPosition(WobbleArmClass.gripperSetReturn());

            //Displaying Telemetry
            telemetry.addData("X Position", OdoClass.odoXReturn());
            telemetry.addData("Y Position", OdoClass.odoYReturn());
            telemetry.addData("Orientation (Degrees)", OdoClass.thetaInDegreesReturn());
            telemetry.addData("theta in Radians", OdoClass.thetaINRadiansReturn());
            telemetry.addData("E1", robot.LF_M.getCurrentPosition());
            telemetry.addData("E2", robot.LB_M.getCurrentPosition());
            telemetry.addData("E3", robot.RF_M.getCurrentPosition());
            telemetry.addData("WB_PT", robot.WB_PT.getVoltage());
            telemetry.addData("WBMotorPower", robot.WB_M.getPower());
            telemetry.addData("ShooterMotorEncoder", robot.SOT_M.getCurrentPosition());
            telemetry.addData("ShooterPowerSet", (ShooterClass.shooterMotorPowerReturn()));
            telemetry.addData("LFM", robot.LF_M.getPower());
            telemetry.addData("SOTAnglePower", ShooterClass.sotAnglePowerReturn());
            telemetry.addData("SOTAngleSet", ShooterClass.sotAngleSetReturn());
            telemetry.addData("SOT_PT", robot.SOT_PT.getVoltage());
            telemetry.addData("Ring1RedValue", Ring1Color.red);
            telemetry.addData("Ring2RedValue", Ring2Color.red);
            telemetry.addData("Ring3RedValue", Ring3Color.red);
            telemetry.update();
        } }}