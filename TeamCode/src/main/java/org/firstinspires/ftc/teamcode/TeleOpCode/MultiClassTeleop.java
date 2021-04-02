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
public class MultiClassTeleop extends LinearOpMode {
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
    double thetaInitial; double justTurn = 0; double thetaSetpoint;
    double intakeSet; boolean shootMethod = false;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    boolean breakOut = false;
    boolean topGoalShoot = false; boolean topGoalOnce;
    double topGoalXPos; double topGoalYPos; double topGoalThetaPos;
    double topGoalXStart; double topGoalYStart;
    boolean topGoalLoop = false;
    @Override
    public void runOpMode() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        //Calling upon the HardwareMap
        robot.init(hardwareMap);
        //setting gain on color sensors
        robot.Ring1_CS.setGain(15); robot.Ring2_CS.setGain(15); robot.Ring3_CS.setGain(15);


        waitForStart();//Waits for the play button to be pressed

        while (opModeIsActive()) {//Main loop that our TeleOp loops in
            //takes color sensor readings in normalized colors so we can get certain color values
            NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors();
            NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors();
            NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();
            //Calling to the Classes and the methods inside of them to run the calculations and set points.
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
         /*   if(gamepad1.x){
                topGoalXPos = OdoClass.odoXReturn(); topGoalYPos = OdoClass.odoYReturn(); topGoalThetaPos = OdoClass.thetaInDegreesReturn();
            }*/
            if (gamepad1.y && !powershotLoop) {//our one button system to control if we are going to run our powershot autonomous section
                if (powershotControl) {
                    powershotMovement = 4;
                } else {
                    powershotControl = true;
                }
                powershotLoop = true;
            } else if (!gamepad1.y) {
                powershotLoop = false;
            }
          /*  if (gamepad1.right_trigger > .05 && !topGoalLoop) {//our one button system to control if we are going to run our powershot autonomous section
                if (topGoalShoot) {
                    topGoalShoot = false;
                } else {
                    topGoalShoot = true;
                }
                topGoalLoop = true;
            } else if (gamepad1.right_trigger < .05) {
                topGoalLoop = false;
            }*/
            if (powershotControl) {
                if (powershotOnlyOnce) {//runs these 3 lines of code that set follow positions only once to ensure we are following the correct position
                    initialPositionX = OdoClass.odoXReturn();
                    initialPositionY = OdoClass.odoYReturn();
                    thetaInitial = OdoClass.thetaInDegreesReturn();
                    powershotOnlyOnce = false;
                    powershotStateOnce = true;
                    powershotMovement = 1;
                }
                if (powershotMovement == 1) {
                    if (powershotStateOnce) {//runs these 3 lines of code to tell us how to get to our endpoint
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false;
                        shootMethod = false;
                        powershotShootOnce = true;//makers sure we run the correct things in the next loop cycles
                        powershotPositionY = initialPositionY + 5;
                        powershotPositionX = initialPositionX;//sets endpoints and the line to follow
                        breakOut = false; //This lets us calculate where the robot is before telling the robot to stop if withing a certain distance
                    }
                    if (DirectionClass.distanceFromReturn() < .5 && (OdoClass.thetaInDegreesReturn() < (thetaInitial + 1) && OdoClass.thetaInDegreesReturn() > thetaInitial - 1) && breakOut) {//TODO add theta correct //once we are within 1 inch of our target position we shoot for 1.5 seconds then move on the 2nd powershot
                        shootMethod = true;
                    } else {
                        breakOut = true;
                    }
                    if (shootMethod) {
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.125, .3, 3, 2.5, 0, .3, 1);//holds position to ensure accuracy when shooting
                        if (powershotShootOnce) {//set the current time when we start the shooting process
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        }
                        if (timerStart + .5 > getRuntime()) {//shoots the ring
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                        } else if (timerStart + .5 < getRuntime()) {
                            RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            powershotMovement = 2;
                            powershotStateOnce = true;
                        }
                    } else {//code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.125, .3, 3, 2.5, 0, .3, 1);
                        RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);//makes sure we don't shoot the ring when we don't want to
                    }
                } else if (powershotMovement == 2) {
                    if (powershotStateOnce) {//runs these 3 lines of code to tell us how to get to our endpoint
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false;
                        shootMethod = false;
                        powershotShootOnce = true;//makers sure we run the correct things in the next loop cycles
                        powershotPositionY = initialPositionY + 11;
                        powershotPositionX = initialPositionX;//sets endpoints and the line to follow
                        breakOut = false; //This lets us calculate where the robot is before telling the robot to stop if withing a certain distance
                    }
                    if (DirectionClass.distanceFromReturn() < .5 && (OdoClass.thetaInDegreesReturn() < (thetaInitial + 1) && OdoClass.thetaInDegreesReturn() > thetaInitial - 1) && breakOut) {//TODO add theta correct //once we are within 1 inch of our target position we shoot for 1.5 seconds then move on the 2nd powershot
                        shootMethod = true;
                    } else {
                        breakOut = true;
                    }
                    if (shootMethod) {
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.125, .3, 3, 2.5, 0, .3, 1);//holds position to ensure accuracy when shooting
                        if (powershotShootOnce) {//set the current time when we start the shooting process
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        }
                        if (timerStart + .5 > getRuntime()) {//shoots the ring
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                        } else if (timerStart + .5 < getRuntime()) {
                            RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            powershotMovement = 3;
                            powershotStateOnce = true;
                        }
                    } else {//code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.125, .3, 3, 2.5, 0, .3, 1);
                        RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);//makes sure we don't shoot the ring when we don't want to
                    }
                } else if (powershotMovement == 3) {
                    if (powershotStateOnce) {//runs these 3 lines of code to tell us how to get to our endpoint
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        powershotStateOnce = false;
                        shootMethod = false;
                        powershotShootOnce = true;//makers sure we run the correct things in the next loop cycles
                        powershotPositionY = initialPositionY + 17;
                        powershotPositionX = initialPositionX;//sets endpoints and the line to follow
                        breakOut = false; //This lets us calculate where the robot is before telling the robot to stop if withing a certain distance
                    }
                    if (DirectionClass.distanceFromReturn() < .5 && (OdoClass.thetaInDegreesReturn() < (thetaInitial + 1) && OdoClass.thetaInDegreesReturn() > thetaInitial - 1) && breakOut) {//TODO add theta correct //once we are within 1 inch of our target position we shoot for 1.5 seconds then move on the 2nd powershot
                        shootMethod = true;
                    } else {
                        breakOut = true;
                    }
                    if (shootMethod) {
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.125, .3, 3, 2.5, 0, .3, 1);//holds position to ensure accuracy when shooting
                        if (powershotShootOnce) {//set the current time when we start the shooting process
                            timerStart = getRuntime();
                            powershotShootOnce = false;
                        }
                        if (timerStart + .5 > getRuntime()) {//shoots the ring
                            RingClass.RingSystemAuto(2, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                        } else if (timerStart + .5 < getRuntime()) {
                            RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);
                            powershotMovement = 4;
                            powershotStateOnce = true;
                        }
                    } else {//code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.125, .3, 3, 2.5, 0, .3, 1);
                        RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);//makes sure we don't shoot the ring when we don't want to
                    }
                } else if (powershotMovement == 4) {
                    powershotControl = false;
                    powershotOnlyOnce = true;
                }
                //runs the shooter to always be running so we keep constant speed
                ShooterClass.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 1700, 1.22);
                WobbleArmClass.WobbleControl(gamepad1.left_trigger, robot.WB_PT.getVoltage());
                //sets drive motors if we are in the powershot subsytem because we control the robot differently than TeleOp
                robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed + .1));
                /*TOP GOAL*//*}else if(topGoalShoot){
                if(topGoalOnce) {
                    topGoalYStart = OdoClass.odoYReturn(); topGoalXStart = OdoClass.odoXReturn();
                    topGoalOnce = false;
                }else{
                    if(DirectionClass.distanceFromReturn() < 2 && (Math.abs(thetaInitial - OdoClass.thetaInDegreesReturn()) < 1)){
                        topGoalShoot = false;
                    }
                }
                RingClass.RingSystemControl(gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.a, gamepad1.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad1.back, ShooterClass.sotAngleSetReturn());
                ShooterClass.shooterControl(gamepad1.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), RingClass.intakePowerReturn());
                Movement(topGoalXPos,topGoalYPos,topGoalThetaPos,50,.3,4,.5,0,5,1.5);
                robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed + .1));*/
            }else{
                //runs the calculations for TeleOp if we are not in the powershot subsytem to let us run a normal TeleOp
                DrivetrainClass.DriveBase(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper);
                RingClass.RingSystemControl(gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.a, gamepad1.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad1.back, ShooterClass.sotAngleSetReturn());
                ShooterClass.shooterControl(gamepad1.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), RingClass.intakePowerReturn());
                WobbleArmClass.WobbleControl(gamepad1.left_trigger, robot.WB_PT.getVoltage());
                //sets motor power if we are not in the powershot subsystem because we control the motor power differently
                robot.LF_M.setPower(DrivetrainClass.LFMReturn());
                robot.LB_M.setPower(DrivetrainClass.LBMReturn());
                robot.RF_M.setPower(DrivetrainClass.RFMReturn());
                robot.RB_M.setPower(DrivetrainClass.RBMReturn());

            }
            if(Ring1Color.red > .05 && Ring2Color.red < .25 && Ring3Color.red < .25){
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;//one ring
            }else if(Ring1Color.red > .05 && Ring2Color.red > .25 && Ring3Color.red < .25){
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;//2 rings
            }else if(Ring1Color.red > .05 && Ring2Color.red > .25 && Ring3Color.red > .25){
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;//3 rings
            }else {
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

            //Displaying Telemetry
            telemetry.addData("intakeSet", intakeSet);
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
        }blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);

    }
    //calls all the methods we need to control the robot autonomously for powershot shooting
    public void Movement (double endpointx, double endpointy, double thetasetpoint, double targetspeed, double thetaTargetSpeed, double thetaDeccelerationDegree,double slowMoveSpeed, double accelerationdistance, double deccelerationdistance, double slowmovedistance){
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        TurnControl.turnControl(thetaSetpoint , OdoClass.thetaInDegreesReturn());
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, slowmovedistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn(), slowMoveSpeed, thetaDeccelerationDegree, thetasetpoint, thetaTargetSpeed, OdoClass.thetaInDegreesReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), OdoClass.thetaInDegreesReturn(), getRuntime(), SpeedClass.speedSetpoint, SpeedClass.thetaSpeedSetpoint());
    }}