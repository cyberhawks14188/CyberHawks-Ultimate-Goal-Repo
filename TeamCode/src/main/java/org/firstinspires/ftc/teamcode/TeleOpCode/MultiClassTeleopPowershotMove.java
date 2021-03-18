package org.firstinspires.ftc.teamcode.TeleOpCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class MultiClassTeleopPowershotMove extends LinearOpMode {
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

    boolean powershotControl, powershotShootOnce;
    double startPointX, startPointY;
    double powershotMovement;
    double powershotPositionY, powershotPositionX;
    double initialPositionX, initialPositionY;
    boolean powershotOnlyOnce = true;
    boolean powershotStateOnce = true;
    double noDriveMotor = 0, timerStart;
    boolean powershotLoop = false;
    double thetaInitial; double justTurn = 0;

    @Override
    public void runOpMode() {
        //Calling upon the HardwareMap
        robot.init(hardwareMap);
        //setting gain on color sensors
        robot.Ring1_CS.setGain(15); robot.Ring2_CS.setGain(15); robot.Ring3_CS.setGain(15);

        waitForStart();//Waits for the play button to be pressed

        while (opModeIsActive()) {//Main loop that our TeleOp loops in
            //takes color sensor readings in normalized colors so we can get certain color values
            NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors(); NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors(); NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();
            //Calling to the Classes and the methods inside of them to run the calculations and set points.
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
            if (gamepad1.right_trigger > .05 && !powershotLoop) {//our one button system to control if we are going to run our powershot autonomous section
                if(powershotControl){
                    powershotControl = false;
                } else {
                    powershotControl = true;
                }
                powershotLoop = true;
            } else if (gamepad1.right_trigger < .05) {
                powershotLoop = false;
            }
            if (powershotControl) {//autonomous powershot code
                if (powershotOnlyOnce) {//runs these 3 lines of code that set follow positions only once to ensure we are following the correct position
                    initialPositionX = OdoClass.odoXReturn(); initialPositionY = OdoClass.odoYReturn(); thetaInitial = OdoClass.thetaInDegreesReturn();
                    powershotOnlyOnce = false; powershotStateOnce = true;
                    powershotMovement = 1;
                }
                if (powershotMovement == 1) {//1st powershot subsystem
                    if (powershotStateOnce) {//runs these 3 lines of code to tell us how to get to our endpoint
                        startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false; powershotShootOnce = true;//makers sure we run the correct things in the next loop cycles
                        powershotPositionY = initialPositionY + 5; powershotPositionX = initialPositionX;//sets endpoints and the line to follow
                    }
                    if (DirectionClass.distanceFromReturn() < 1) {//once we are within 1 inch of our target position we shoot for 1.5 seconds then move on the 2nd powershot
                        Movement(powershotPositionX, powershotPositionY, thetaInitial,10, 0, 4);//holds position to ensure accuracy when shooting
                        if (powershotShootOnce) {//set the current time when we start the shooting process
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        }
                        if (timerStart + 1.5 > getRuntime()) {//shoots the ring
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                        } else if (timerStart + 1.5 < getRuntime()) {
                            RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            powershotMovement = 2; powershotStateOnce = true;
                        }
                    }else{//code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 20, 0, 4);
                        RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);//makes sure we don't shoot the ring when we don't want to
                    }
                } else if (powershotMovement == 2) {//2nd powershot subsystem
                    if (powershotStateOnce) {//runs these 3 lines of code to tell us how to get to our endpoint
                        startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false; powershotShootOnce = true;//makers sure we run the correct things in the next loop cycles
                        powershotPositionY = initialPositionY + 10; powershotPositionX = initialPositionX;//sets endpoints and the line to follow
                    }
                    if (DirectionClass.distanceFromReturn() < 1) {//once we are within 1 inch of our target position we shoot for 1.5 seconds then move on the 2nd powershot
                        Movement(powershotPositionX, powershotPositionY, thetaInitial,10, 0, 4);//holds position to ensure accuracy when shooting
                        if (powershotShootOnce) {//set the current time when we start the shooting process
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        }
                        if (timerStart + 1.5 > getRuntime()) {//shoots the ring
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                        } else if (timerStart + 1.5 < getRuntime()) {
                            RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            powershotMovement = 3; powershotStateOnce = true;
                        }
                    }else{//code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 20, 0, 4);
                        RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                    }
                }else if (powershotMovement == 3) {//3rd powershot subsystem
                    if (powershotStateOnce) {//runs these 3 lines of code to tell us how to get to our endpoint
                        startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false; powershotShootOnce = true;
                        powershotPositionY = initialPositionY + 15; powershotPositionX = initialPositionX;
                    }
                    if (DirectionClass.distanceFromReturn() < 1) {//once we are within 1 inch of our target position we shoot for 1.5 seconds then move on the 2nd powershot
                        Movement(powershotPositionX, powershotPositionY, thetaInitial,10, 0, 4);//holds position to ensure accuracy when shooting
                        if (powershotShootOnce) {//set the current time when we start the shooting process
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        }
                        if (timerStart + 1.5 > getRuntime()) {//shoots the ring
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                        } else if (timerStart + 1.5 < getRuntime()) {powershotControl = false;}//gets out of the powershot subsystem to continue teleOp
                    }else{//code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial,20, 0, 4);
                        RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                    }
                }
                //runs the shooter to always be running so we keep constant speed
                ShooterClass.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 1900, 1.275);
                //sets drive motors if we are in the powershot subsytem because we control the robot differently than TeleOp
                robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .35));
                robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed + .35));
                robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed + .35));
                robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed + .35));
            }else{
                //runs the calculations for TeleOp if we are not in the powershot subsytem to let us run a normal TeleOp
                DrivetrainClass.DriveBase(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper);
                RingClass.RingSystemControl(gamepad1.a, gamepad1.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad1.back, ShooterClass.sotAngleSetReturn());
                ShooterClass.shooterControl(gamepad1.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), RingClass.intakePowerReturn());
                WobbleArmClass.WobbleControl(gamepad1.left_trigger, robot.WB_PT.getVoltage());
                //sets motor power if we are not in the powershot subsystem because we control the motor power differently
                robot.LF_M.setPower(DrivetrainClass.LFMReturn());
                robot.LB_M.setPower(DrivetrainClass.LBMReturn());
                robot.RF_M.setPower(DrivetrainClass.RFMReturn());
                robot.RB_M.setPower(DrivetrainClass.RBMReturn());
                robot.IN_M.setPower(RingClass.intakePowerReturn());
            }
            //sets universal motor power
            robot.WB_M.setPower(WobbleArmClass.wobblePowerReturn());
            robot.GRIP_S.setPosition(WobbleArmClass.gripperSetReturn());
            robot.SOT_M.setPower(ShooterClass.shooterMotorPowerReturn());
            robot.SOT_S.setPower(ShooterClass.sotAnglePowerReturn());
            robot.STG_M.setPower(RingClass.stagerPowerRetun());
            robot.STOP_S.setPosition(RingClass.stopperSetReturn());
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
        }}
        //calls all the methods we need to control the robot autonomously for powershot shooting
    public void Movement ( double endpointx, double endpointy, double thetasetpoint, double targetspeed, double accelerationdistance, double deccelerationdistance){
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
       // DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), getRuntime(), SpeedClass.speedSetpoint);
        if(justTurn == 1){
            TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(), 3);
        }
        else{
            TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(), 1);
        }
        telemetry.addData("Speed Setpoint", SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn()));
    }}