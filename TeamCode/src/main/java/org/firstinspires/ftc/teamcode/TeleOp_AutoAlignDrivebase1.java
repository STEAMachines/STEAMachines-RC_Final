//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@TeleOp(name="TeleOp_AutoAlign", group="STEAMachines_DECODE")
//public class TeleOp_AutoAlignDrivebase1 extends LinearOpMode {
//    DcMotorEx leftDrive;
//    DcMotorEx rightDrive;
//    DcMotorEx launcherMotors;
//
//    @Override
//    public void runOpMode() {
//        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
//        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
//        launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");
//        waitForStart();
//        while(opModeIsActive()) {
//            double forward = -gamepad1.left_stick_y;
//            double turn = gamepad1.right_stick_x;
//
//            boolean targetVisible = false;
//            double targetYaw = 0.0;
//        }
//    }
//}
