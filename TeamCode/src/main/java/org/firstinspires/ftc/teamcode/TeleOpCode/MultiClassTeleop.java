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

    //declares our variable types to let us read our code easier
    boolean powershotControl, powershotShootOnce;
    double startPointX, startPointY;
    double powershotMovement;
    double powershotPositionY, powershotPositionX;
    double initialPositionX, initialPositionY;
    boolean powershotOnlyOnce = true;
    double timerStart;
    boolean powershotLoop = false;
    double thetaInitial;
    boolean shootMethod = false;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    boolean breakOut = false;
    boolean topGoalShoot = false; boolean topGoalOnce = true;
    double topGoalXPos; double topGoalYPos; double topGoalThetaPos;
    double topGoalXStart; double topGoalYStart;
    boolean topGoalLoop = false;
    @Override
    public void runOpMode() {
        //calls the Blinkin LED Driver to let us change the LED Colors in TeleOp
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        //Calling upon the HardwareMap
        robot.init(hardwareMap);
        //setting gain on color sensors for more accurate readings
        robot.Ring1_CS.setGain(15); robot.Ring2_CS.setGain(15); robot.Ring3_CS.setGain(15);


        waitForStart();//Waits for the play button to be pressed

        while (opModeIsActive()) {//Main loop that our TeleOp loops in
            //takes color sensor readings in normalized colors so we can get specific color values
            NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors();
            NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors();
            NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();

            //Calling the odometry class to let us calculate where we are for later use
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());

            //when x button is pressed we set our current position to the position we go to in our go to top goal position subsystem
            if(gamepad1.x){
                topGoalXPos = OdoClass.odoXReturn(); topGoalYPos = OdoClass.odoYReturn(); topGoalThetaPos = OdoClass.thetaInDegreesReturn();
            }

            //uses our custom 1 button subsystem to set the variable to run our powershot subsystem
            if (gamepad1.y && !powershotLoop) {
                if (powershotControl) {
                    powershotMovement = 4;
                } else {
                    powershotControl = true;
                }
                powershotLoop = true;
            } else if (!gamepad1.y) {
                powershotLoop = false;
            }

            //uses our custom 1 button subsystem to set the variable to run our go to top goal shooting position subsystem
            if (gamepad1.right_trigger > .05 && !topGoalLoop) {
                if (topGoalShoot) {
                    topGoalShoot = false;
                    topGoalOnce = true;
                } else {
                    topGoalShoot = true;
                }
                topGoalLoop = true;
            } else if (gamepad1.right_trigger < .05) {
                topGoalLoop = false;
            }

            //This if do controls weather we are in our powershot subsystem, going to the top goal or just normal TeleOp
            if (powershotControl) {//our subsystem that automatically shoots all 3 powershots

                if (powershotOnlyOnce) {//this if do runs only once when we enter the subsystem and sets everything to ensure the subsystem works correctly and goes the the correct spot
                    initialPositionX = OdoClass.odoXReturn(); initialPositionY = OdoClass.odoYReturn();
                    thetaInitial = OdoClass.thetaInDegreesReturn();//sets our current position to a varibale to calculate our target position later
                    powershotOnlyOnce = false; powershotMovement = 1;
                    startPointX = OdoClass.odoXReturn();startPointY = OdoClass.odoYReturn();
                    shootMethod = false;
                    powershotShootOnce = true;
                    powershotPositionY = initialPositionY + 8.5; powershotPositionX = initialPositionX;//sets endpoints and the line to follow
                    breakOut = false;//makes sure that our distance from variable has time to calculate before entering the shooting method
                }
                if (powershotMovement == 1) {//our 1st powershot sub section
                    if (DirectionClass.distanceFromReturn() < .5 && (OdoClass.thetaInDegreesReturn() < (thetaInitial + 1) && OdoClass.thetaInDegreesReturn() > thetaInitial - 1) && breakOut) {//if within .5 in and 1 degree ni each direction then shoot a ring
                        //once we are in our designated target position run the shoot method to shoot 1 ring
                        shootMethod = true;
                    } else {
                        //makes sure that between each loop cycle the hub can calculate our distance from to ensure we go to the next position
                        breakOut = true;
                    }
                    if (shootMethod) {
                        //runs our method to shoot 1 ring even if the robot gets out of the target zone a little bit
                        shootSubsystem(Ring1Color.red, Ring2Color.red, Ring3Color.red, 15.5, 2);
                    } else {
                        //code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.5, .3, 3, 3, 0, .3, 1);
                        RingClass.RingSystemAuto(1, Ring1Color.red, Ring2Color.red, Ring3Color.red);//makes sure we don't shoot the ring when we don't want to
                    }
                } else if (powershotMovement == 2) {
                    if (DirectionClass.distanceFromReturn() < .5 && (OdoClass.thetaInDegreesReturn() < (thetaInitial + 1) && OdoClass.thetaInDegreesReturn() > thetaInitial - 1) && breakOut) {//if within .5 in and 1 degree ni each direction then shoot a ring
                        //once we are in our designated target position run the shoot method to shoot 1 ring
                        shootMethod = true;
                    } else {
                        //makes sure that between each loop cycle the hub can calculate our distance from to ensure we go to the next position
                        breakOut = true;
                    }
                    if (shootMethod) {
                        //runs our method to shoot 1 ring even if the robot gets out of the target zone a little bit
                        shootSubsystem(Ring1Color.red, Ring2Color.red, Ring3Color.red, 22.5, 3);
                    } else {
                        //code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.5, .3, 3, 3, 0, .3, 1);
                        RingClass.RingSystemAuto(0, Ring1Color.red, Ring2Color.red, Ring3Color.red);//makes sure we don't shoot the ring when we don't want to
                    }
                } else if (powershotMovement == 3) {
                    if (DirectionClass.distanceFromReturn() < .5 && (OdoClass.thetaInDegreesReturn() < (thetaInitial + 1) && OdoClass.thetaInDegreesReturn() > thetaInitial - 1) && breakOut) {//if within .5 in and 1 degree ni each direction then shoot a ring
                        //once we are in our designated target position run the shoot method to shoot 1 ring
                        shootMethod = true;
                    } else {
                        //makes sure that between each loop cycle the hub can calculate our distance from to ensure we go to the next position
                        breakOut = true;
                    }
                    if (shootMethod) {
                        //runs our method to shoot 1 ring even if the robot gets out of the target zone a little bit
                        shootSubsystem(Ring1Color.red, Ring2Color.red, Ring3Color.red, 12, 4);
                    } else {
                        //code to get us to our target position
                        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.5, .3, 3, 3, 0, .3, 1);
                        RingClass.RingSystemAuto(1, Ring1Color.red, Ring2Color.red, Ring3Color.red);//makes sure we don't shoot the ring when we don't want to
                    }
                } else if (powershotMovement == 4) {
                    //we run this every time we press run the powershot subsytem to make sure we run the correct things
                    powershotControl = false;
                    powershotOnlyOnce = true;
                }
                //runs the shooter to always be running so we keep constant speed
                ShooterClass.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 1300, .97);
                //holds wobble goal position so the arm holds a wobble goal if we have one
                WobbleArmClass.WobbleControl(gamepad1.left_trigger, robot.WB_PT.getVoltage());
                //sets drive motors if we are in the powershot subsytem because we control the robot differently than TeleOp
                robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed + .1));
            }else if(topGoalShoot){
                //This is our go to powershot subsystem to go to shooting position faster
                if(topGoalOnce) {
                    //this runs once per time we call it to set our start position to accuratly move to position
                    topGoalYStart = OdoClass.odoYReturn(); topGoalXStart = OdoClass.odoXReturn();
                    topGoalOnce = false;
                }else{
                    //Turns of the go to position once within our desired position
                    if(DirectionClass.distanceFromReturn() < .5 && (OdoClass.thetaInDegreesReturn() < (thetaInitial + 1) && OdoClass.thetaInDegreesReturn() > thetaInitial - 1)){
                        topGoalShoot = false;
                    }
                }
                //sets and calls motor power calculations to still run all of our subsystems while moving to our next position
                RingClass.RingSystemControl(gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.a, gamepad1.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad1.back, ShooterClass.sotAngleSetReturn());
                ShooterClass.shooterControl(gamepad1.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), RingClass.intakePowerReturn());
                Movement(topGoalXPos,topGoalYPos,topGoalThetaPos,50,.3,4,.5,0,5,1.5);
                robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed + .1));
                robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed + .1));
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
            //Sets our LEDs to different colors depending on if how many rings we have in the robot
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
    //declare the movement method that we use for autonomously moving th robot in the powershot and top goal subsystem
    public void Movement (double endpointx, double endpointy, double thetasetpoint, double targetspeed, double thetaTargetSpeed, double thetaDeccelerationDegree,double slowMoveSpeed, double accelerationdistance, double deccelerationdistance, double slowMovedDistance){
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        TurnControl.turnControl(thetasetpoint , OdoClass.thetaInDegreesReturn());
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, slowMovedDistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn(), slowMoveSpeed, thetaDeccelerationDegree, thetasetpoint, thetaTargetSpeed, OdoClass.thetaInDegreesReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), OdoClass.thetaInDegreesReturn(), getRuntime(), SpeedClass.speedSetpoint(), SpeedClass.thetaSpeedSetpoint());
    }
    //declares our shootings subsystem for our powershot method
    public void shootSubsystem (double color1, double color2, double color3, double nextypos, double nextmove){
        Movement(powershotPositionX, powershotPositionY, thetaInitial, 7.5, .3, 3, 3, 0, .3, 1);//holds position to ensure accuracy when shooting
        if (powershotShootOnce) {//set the current time when we start the shooting process
            timerStart = getRuntime();
            powershotShootOnce = false;
        }
        if (timerStart + .5 > getRuntime()) {//shoots the ring
            RingClass.RingSystemAuto(2, color1, color2, color3);
        } else if (timerStart + .5 < getRuntime()) {
            RingClass.RingSystemAuto(0, color1, color2, color3);
            powershotMovement = nextmove;
            startPointX = OdoClass.odoXReturn();
            startPointY = OdoClass.odoYReturn();
            shootMethod = false;
            powershotShootOnce = true;//makers sure we run the correct things in the next loop cycles
            powershotPositionY = initialPositionY + nextypos;
            powershotPositionX = initialPositionX;//sets endpoints and the line to follow
            breakOut = false; //This lets us calculate where the robot is before telling the robot to stop if withing a certain distance

        }
    }
}