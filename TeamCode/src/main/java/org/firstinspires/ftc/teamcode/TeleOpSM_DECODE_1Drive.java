package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@Disabled
@TeleOp(name="TeleOpSM_DECODE-1Drivers", group="STEAMachines_DECODE")
public class TeleOpSM_DECODE_1Drive extends LinearOpMode {
    final double DESIRED_DISTANCE = 12.0;
    final double SPEED_GAIN = 0.02;
    final double TURN_GAIN = 0.01;
    final double MAX_AUTO_SPEED = 1.0;
    final double MAX_AUTO_TURN = 0.5;
    public static double p = 10;
    public static double i = 1.5;
    public static double d = 0;
    public static double f = 12.5;
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private DcMotor intakeMotors;
    public static DcMotorEx launcherMotors;
    private Servo handleServo;
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection desiredTag;
//    private VisionPortal visionPortal;
//    private static final boolean USE_WEBCAM = true;
//    private static final int DESIRED_TAG_ID = 20;
//    boolean intakeToggle = false;

    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        leftDrive = hardwareMap.get(DcMotorEx.class,"leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        intakeMotors = hardwareMap.get(DcMotor.class, "intakeMotors");
        launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");
        handleServo = hardwareMap.get(Servo.class, "handleServo");
//        leftDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        launcherMotors.setDirection(DcMotor.Direction.REVERSE);
        PIDFCoefficients pidf = launcherMotors.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
//        initializeAprilTag();
//        displayWebcamVision();
        while(opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));

            double leftPower;
            double rightPower;
            double max_ticks = 2500.0;
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(turn + drive, -1.0, 1.0);
            rightPower = Range.clip(turn - drive, -1.0, 1.0);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            //left_bumper && right_bumper to move......
            if(gamepad1.right_bumper) {
                intakeMotors.setPower(-1);
            }
            else if(gamepad1.left_bumper) {
                intakeMotors.setPower(1);
            }
            else {
                intakeMotors.setPower(0);
            }

//            if (currentGamepad1.left_bumper && !currentGamepad1.left_bumper) {
//                intakeToggle = !intakeToggle;
//            }
//            if (intakeToggle) {
//                intakeMotors.setPower(1);
//            }
//            else {
//                intakeMotors.setPower(0);
//            }
            //left_trigger && right_trigger to move..
            if(gamepad1.right_trigger == 1.0) {
                launcherMotors.setVelocity(1650);
                launcherMotors.setDirection(DcMotorEx.Direction.REVERSE);
            }
            else if(gamepad1.y) {
                launcherMotors.setVelocity(1650);
                launcherMotors.setDirection(DcMotorEx.Direction.FORWARD);
            }
//            else if(gamepad1.right_trigger == 1.0) {
//                launcherMotors.setVelocity(0);
//            }
            //Eksperimen pake else-if
            else {
                launcherMotors.setVelocity(0);
            }
            if(gamepad1.b) {
                handleServo.setPosition(0);
            }
            else if(gamepad1.a) {
                handleServo.setPosition(0.6);
            }
            telemetry.addLine("STEAMachines_DECODE");
            telemetry.addData("Drivebase-Left:", leftDrive.getPower());
            telemetry.addData("Drivebase-Right:", rightDrive.getPower());
            telemetry.addData("Shooter (Velocity):", launcherMotors.getVelocity());
            telemetry.addData("Shooter (P)", pidf.p);
            telemetry.addData("Shooter (I)", pidf.i);
            telemetry.addData("Shooter (D)", pidf.d);
            telemetry.addData("Shooter (F)", pidf.f);
            telemetry.addData("Intake:", intakeMotors.getPower());
            telemetry.update();
        }
    }
//    public void initializeAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder().build();
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        if(USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "WebcamSM"));
//        }
//        else {
//            builder.setCamera(BuiltinCameraDirection.FRONT);
//        }
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
//    }
//    public void displayWebcamVision() {
//        boolean targetFound = true;
//        double drive = 0;
//        double turn = 0;
//        while(opModeIsActive()) {
//            List<AprilTagDetect   ion> currentDetections = aprilTag.getDetections();
//            for(AprilTagDetection detection:currentDetections) {
//                if (detection.metadata != null) {
//                    if(DESIRED_TAG_ID < 0 || (detection.id == DESIRED_TAG_ID)) {
//                        targetFound = true;
//                        desiredTag = detection;
//                        break;
//                    }
//                    else {
//                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                    }
//                }
//                else {
//                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//                }
//                telemetry.addData("ID", detection.id);
//                telemetry.addData("Range", detection.ftcPose.range);
//                telemetry.addData("Bearing", detection.ftcPose.bearing);
//            }
//            if(targetFound) {
//                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//                double headingError = (desiredTag.ftcPose.bearing);
//                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
//                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
//            }
//            else {
//                targetFound = false;
//            }
//            telemetry.update();
//        }
//    }
}
