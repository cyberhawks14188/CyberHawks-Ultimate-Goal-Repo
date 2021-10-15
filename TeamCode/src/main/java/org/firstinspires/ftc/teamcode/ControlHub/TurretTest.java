package org.firstinspires.ftc.teamcode.ControlHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TurretTest extends LinearOpMode {
    JustMotorRobotHardware robot = new JustMotorRobotHardware();


    double extendMin = 2, extendMax = 100, extendSet = extendMin;
    double extendDifference = 0, extendMultiplied = 0, extendP = .0001, extendD = 0;


    double vPivotMin = 0, vPivotMax = 3, vPivotSet = .5;
    double vPivotDifference = 0, vPivotMultiplied = 0, vPivotP = .0001, vPivotD = 0;

    double rotateMin = -270, rotateMax = 270, rotateSet = 0;
    double rotateDifference = 0, rotateMultiplied = 0, rotateP = .0001, rotateD = 0;
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            //Linear Slides Extend

            //Setpoint limits
            if(extendSet < extendMin){
                extendSet = extendMin;
            }else  if( extendSet > extendMax){
                extendSet = extendMax;
            }

            extendDifference = robot.Motor1.getCurrentPosition() - extendSet;
            extendMultiplied = extendDifference * extendP;

            //VPivot Limits
            if(vPivotSet < vPivotMin){
                vPivotSet = vPivotMin;
            }else if(vPivotSet > vPivotMax){
                vPivotSet = vPivotMax;
            }

            vPivotDifference = robot.PivotPT.getVoltage() - vPivotSet;
            vPivotMultiplied = vPivotDifference * vPivotP;


            //Hpivot Limits
            if(rotateSet < rotateMin){
                rotateSet = rotateMin;
            }else if(rotateSet > rotateMax){
                rotateSet = rotateMax;
            }

            rotateDifference = robot.Motor3.getCurrentPosition() - rotateSet;
            rotateMultiplied = rotateDifference * rotateP;
/*
            robot.Motor1.setPower(extendMultiplied);
            robot.Motor2.setPower(vPivotMultiplied);
            robot.Motor3.setPower(rotateMultiplied);
            */

            telemetry.addData("motor1", robot.Motor1.getPower());
            telemetry.addData("rotate power", rotateMultiplied);
            telemetry.addData("Vpivot power", vPivotMultiplied);
            telemetry.addData("extend power", extendMultiplied);
            telemetry.update();
        }
    }
}
