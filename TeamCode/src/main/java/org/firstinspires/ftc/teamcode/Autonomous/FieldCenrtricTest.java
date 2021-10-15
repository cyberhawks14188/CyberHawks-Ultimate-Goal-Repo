package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.VectorDriveClass;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.Odometry;

@TeleOp
public class FieldCenrtricTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    VectorDriveClass VectorDriveClass = new VectorDriveClass();
    Odometry OdoClass = new Odometry();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            VectorDriveClass.VectorDriveMethod(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, OdoClass.thetaInDegreesReturn());
            robot.RF_M.setPower(VectorDriveClass.RF_MRETURN());
            robot.RB_M.setPower(VectorDriveClass.RB_MRETURN());
            robot.LF_M.setPower(VectorDriveClass.LF_MRETURN());
            robot.LB_M.setPower(VectorDriveClass.LB_MRETURN());
            double tantest = Math.atan(.5);
            telemetry.addData("tantest", Math.toDegrees(tantest));
            telemetry.addData("RF_M", VectorDriveClass.RF_MRETURN());
            telemetry.addData("RB_M", VectorDriveClass.RB_MRETURN());
            telemetry.addData("LF_M", VectorDriveClass.LF_MRETURN());
            telemetry.addData("LB_M", VectorDriveClass.LB_MRETURN());
            telemetry.addData("theta degrees", OdoClass.thetaInDegreesReturn());
            telemetry.update();
        }
    }
}
