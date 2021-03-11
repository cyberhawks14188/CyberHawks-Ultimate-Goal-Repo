package org.firstinspires.ftc.teamcode.TeleOpCode;

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
public class MultiClassTeleop extends LinearOpMode {
    Odometry OdoClass = new Odometry();
    RingSystem RingClass = new RingSystem();
    ShooterSystem ShooterClass = new ShooterSystem();
    WobbleGoalArm WobbleArmClass = new WobbleGoalArm();
    DriveTrain DrivetrainClass = new DriveTrain();
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    SpeedClass SpeedClass = new SpeedClass();
    TurnControl TurnControl = new TurnControl();
    RobotHardware robot = new RobotHardware();


    boolean powershotControl, powershotShootOnce;
    double startPointX, startPointY;
    double powershotMovement;
    double powershotPositionY, powershotPositionX;
    double initialPositionX, initialPositionY;
    boolean powershotOnlyOnce = true;
    boolean powershotStateOnce = true;
    double noDriveMotor = 0, timerStart;
    boolean powershotLoop = false;

    @Override
    public void runOpMode() {
        //Calling upon the HardwareMap and other classes

        robot.init(hardwareMap);
        //setting gain on color sensors
        robot.Ring1_CS.setGain(15);
        robot.Ring2_CS.setGain(15);
        robot.Ring3_CS.setGain(15);

        //Waits for the play button to be pressed
        waitForStart();
        //mMin loop that our TeleOp loops in
        while (opModeIsActive()) {
            NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors();
            NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors();
            NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();
            //Calling to the Classes and the methods inside of them to run the calculations and set points.
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
            if (gamepad1.right_trigger > .05 && !powershotLoop) {
                if(powershotControl){
                    powershotControl = false;
                } else {
                    powershotControl = true;
                }
                powershotLoop = true;
            } else if (gamepad1.right_trigger < .05) {
                powershotLoop = false;
            }
            if (powershotControl) {
                if (powershotOnlyOnce) {
                    initialPositionX = OdoClass.odoXReturn();
                    initialPositionY = OdoClass.odoYReturn();
                    powershotOnlyOnce = false;
                    powershotMovement = 1;
                    powershotStateOnce = true;
                }
                if (powershotMovement == 1) {
                    if (powershotStateOnce) {
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false;
                        powershotShootOnce = true;
                        powershotPositionY = initialPositionY + 5;
                        powershotPositionX = initialPositionX;
                    }
                    Movement(powershotPositionX, powershotPositionY, 0, 20, 2, 4);
                    RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                    if (DirectionClass.distanceFromReturn() < 1) {
                        if (powershotShootOnce) {
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        }else if (timerStart + .7 > getRuntime()) {
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            noDriveMotor = 1;
                        } else if (timerStart + .7 < getRuntime()) {
                            RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            powershotMovement = 2; noDriveMotor = 0; powershotStateOnce = true;
                        }
                    }else{
                        noDriveMotor = 0;
                    }
                } else if (powershotMovement == 2) {
                    if (powershotStateOnce) {
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false;
                        powershotShootOnce = true;
                        powershotPositionY = initialPositionY + 10;
                        powershotPositionX = initialPositionX;

                    }
                    Movement(powershotPositionX, powershotPositionY, 0, 20, 2, 4);
                    RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                    if (DirectionClass.distanceFromReturn() < 1) {
                        if (powershotShootOnce) {
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        } else if (timerStart + .7 > getRuntime()) {
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            noDriveMotor = 0;
                        } else if (timerStart + .7 < getRuntime()) {
                            RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            powershotMovement = 3;
                            noDriveMotor = 1;
                            powershotStateOnce = true;
                        }
                    }else{
                        noDriveMotor = 0;
                    }
                }else if (powershotMovement == 3) {
                    if (powershotStateOnce) {
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false;
                        powershotShootOnce = true;
                        powershotPositionY = initialPositionY + 15;
                        powershotPositionX = initialPositionX;
                    }
                    Movement(powershotPositionX, powershotPositionY, 0,20, 2, 4);
                    RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                    if (DirectionClass.distanceFromReturn() < 1) {
                        if (powershotShootOnce) {
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        }else if (timerStart + .7 > getRuntime()) {
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            noDriveMotor = 1;
                        } else if (timerStart + .7 < getRuntime()) {
                            RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            powershotControl = false; noDriveMotor = 0;
                        }
                    }else{
                        noDriveMotor = 0;
                    }
                }


            if (noDriveMotor == 1) {
                robot.WB_M.setPower(WobbleArmClass.wobblePowerReturn());
                robot.GRIP_S.setPosition(WobbleArmClass.gripperSetReturn());
                robot.SOT_M.setPower(ShooterClass.shooterMotorPowerReturn());
                robot.SOT_S.setPower(ShooterClass.sotAnglePowerReturn());
                robot.STG_M.setPower(RingClass.stagerPowerRetun());
                robot.STOP_S.setPosition(RingClass.stopperSetReturn());
                robot.LF_M.setPower(0);
                robot.LB_M.setPower(0);
                robot.RF_M.setPower(0);
                robot.RB_M.setPower(0);
            } else {
                robot.WB_M.setPower(WobbleArmClass.wobblePowerReturn());
                robot.GRIP_S.setPosition(WobbleArmClass.gripperSetReturn());
                robot.SOT_M.setPower(ShooterClass.shooterMotorPowerReturn());
                robot.SOT_S.setPower(ShooterClass.sotAnglePowerReturn());
                robot.STG_M.setPower(RingClass.stagerPowerRetun());
                robot.STOP_S.setPosition(RingClass.stopperSetReturn());
                robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .3));
                robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed + .3));
                robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed + .3));
                robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed + .3));
            }
            ShooterClass.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 1900, 1.275);
        }else{
            DrivetrainClass.DriveBase(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper);
            RingClass.RingSystemControl(gamepad1.a, gamepad1.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad1.back, ShooterClass.sotAngleSetReturn());
            ShooterClass.shooterControl(gamepad1.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), RingClass.intakePowerReturn());
            WobbleArmClass.WobbleControl(gamepad1.left_trigger, gamepad1.dpad_up, gamepad1.dpad_down, robot.WB_PT.getVoltage());
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
        }
        //Setting Motor Power


        //Displaying Telemetry
        telemetry.addData("speed variable", SpeedClass.SpeedReturn());
            telemetry.addData("powershotMovement", powershotMovement);
        telemetry.addData("powershotinitialY", powershotPositionY);
        telemetry.addData("robot.SOT_S.getPower()", robot.SOT_S.getPower());
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
    }

    }

    public void Movement ( double endpointx, double endpointy, double thetasetpoint, double targetspeed, double accelerationdistance, double deccelerationdistance){
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), getRuntime(), SpeedClass.speedSetpoint);
        TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(), 1);

        telemetry.addData("Speed Setpoint", SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn()));
    }

}

