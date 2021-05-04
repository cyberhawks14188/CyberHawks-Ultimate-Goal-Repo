package org.firstinspires.ftc.teamcode.TeleOpCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.DirectionCalcClass;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.SpeedClass;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.TurnControl;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.Odometry;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.RingSystem;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.ShooterSystem;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.WobbleGoalArm;

@TeleOp
public class DEMOTeleop extends LinearOpMode {
    //calls our classes to let us organize our code easier in different classes
    Odometry OdoClass = new Odometry();
    RingSystem RingClass = new RingSystem();
    ShooterSystem ShooterClass = new ShooterSystem();
    WobbleGoalArm WobbleArmClass = new WobbleGoalArm();
    DriveTrain DrivetrainClass = new DriveTrain();
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    SpeedClass SpeedClass = new SpeedClass();
    TurnControl TurnControl = new TurnControl();
    RobotHardware robot = new RobotHardware();

    //declares our variable types to let us read our code easier
    double powershotMovement;
    double powershotPositionY, powershotPositionX;
    double thetaInitial;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    boolean slowOverride = true; boolean forceStop = true;

    @Override
    public void runOpMode() {
        //calls the Blinkin LED Driver to let us change the LED Colors in TeleOp
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        //Calling upon the HardwareMap
        robot.init(hardwareMap);
        //setting gain on color sensors for more accurate readings
        robot.Ring1_CS.setGain(15);
        robot.Ring2_CS.setGain(15);
        robot.Ring3_CS.setGain(15);


        waitForStart();//Waits for the play button to be pressed

        while (opModeIsActive() && forceStop) {//Main loop that our TeleOp loops in
            //takes color sensor readings in normalized colors so we can get specific color values
            NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors();
            NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors();
            NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();

            //Calling the odometry class to let us calculate where we are for later use
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());

            //runs the calculations for TeleOp if we are not in the powershot subsytem to let us run a normal TeleOp

            //runs over ride commands to prevent the the robot from doing something bad

            if(gamepad2.right_trigger > .08){
                DrivetrainClass.DriveBase(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_bumper);
                RingClass.RingSystemControl(gamepad2.dpad_down, gamepad2.dpad_up, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.a, gamepad2.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad2.back, ShooterClass.sotAngleSetReturn());
                ShooterClass.shooterControl(gamepad2.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), RingClass.intakePowerReturn());
                WobbleArmClass.WobbleControl(gamepad1.left_trigger, robot.WB_PT.getVoltage());
            }else{
                DrivetrainClass.DriveBase(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, slowOverride);
                RingClass.RingSystemControl(gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.a, gamepad1.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad1.back, ShooterClass.sotAngleSetReturn());
                ShooterClass.shooterControl(gamepad1.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), RingClass.intakePowerReturn());
                WobbleArmClass.WobbleControl(gamepad1.left_trigger, robot.WB_PT.getVoltage());
            }
            if(gamepad2.y || gamepad2.x){
                forceStop = false;
            }

            robot.LF_M.setPower(DrivetrainClass.LFMReturn());
            robot.LB_M.setPower(DrivetrainClass.LBMReturn());
            robot.RF_M.setPower(DrivetrainClass.RFMReturn());
            robot.RB_M.setPower(DrivetrainClass.RBMReturn());

            //Sets our LEDs to different colors depending on if how many rings we have in the robot
            if (Ring1Color.red > .05 && Ring2Color.red < .25 && Ring3Color.red < .25) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;//one ring
            } else if (Ring1Color.red > .05 && Ring2Color.red > .25 && Ring3Color.red < .25) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;//2 rings
            } else if (Ring1Color.red > .05 && Ring2Color.red > .25 && Ring3Color.red > .25) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;//3 rings
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;//no rings
            }


            //sets universal motor power
            robot.WB_M.setPower(WobbleArmClass.wobblePowerReturn());
            robot.GRIP_S.setPosition(WobbleArmClass.gripperSetReturn());
            robot.SOT_M.setPower(ShooterClass.shooterMotorPowerReturn());
            robot.SOT_S.setPower(ShooterClass.sotAnglePowerReturn());
            robot.STG_M.setPower(RingClass.stagerPowerRetun());
            robot.STOP_S.setPosition(RingClass.stopperSetReturn());
            robot.IN_S.setPosition(RingClass.intakePositionReturn());
            robot.IN_M.setPower(RingClass.intakePowerReturn());
            blinkinLedDriver.setPattern(pattern);

            //Displaying Telemetry to help us figure our if something happened to the robot
            telemetry.addData("distancefrom", DirectionClass.distanceFromReturn());
            telemetry.addData("theta initial", thetaInitial);
            telemetry.addData("Orientation (Degrees)", OdoClass.thetaInDegreesReturn());
            telemetry.addData("speed variable", SpeedClass.SpeedReturn());
            telemetry.addData("powershotMovement", powershotMovement);
            telemetry.addData("powershotinitialY", powershotPositionY);
            telemetry.addData("robot.SOT_S.getPower()", robot.SOT_S.getPower());
            telemetry.addData("X Position", OdoClass.odoXReturn());
            telemetry.addData("Y Position", OdoClass.odoYReturn());
            telemetry.addData("theta in Radians", OdoClass.thetaINRadiansReturn());
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
        }
        //once the program ends we set the light pattern to the default to look consistent
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);

    }
}