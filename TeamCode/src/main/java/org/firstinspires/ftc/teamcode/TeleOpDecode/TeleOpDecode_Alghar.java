package org.firstinspires.ftc.teamcode.TeleOpDecode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name="AutoAlign_TeleOp-Alghar", group="STEAMachines_DECODE")
public class TeleOpDecode_Alghar extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    double speedMultiplier = 0.7;
    public static double launcherPower = 1550; // Power default untuk launcher
    double powerIncrement = 100; // Besaran perubahan power

    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;

    // Toggle launcher variables
    boolean launcherOn = false;
    boolean leftBumperPressed = false;

    // Auto-adjust launcher power berdasarkan jarak
    boolean autoAdjustEnabled = false;
    boolean xButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
//        DcMotorEx intakemotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
//        CRServo intake = hardwareMap.crservo.get("intakeServo");
//        CRServo intake2 = hardwareMap.crservo.get("intake2");
//        DcMotorEx launcher = (DcMotorEx) hardwareMap.dcMotor.get("launcher");
//        Servo pusher = hardwareMap.servo.get("pusher");

        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        DcMotor intakeMotors = hardwareMap.get(DcMotor.class, "intakeMotors");
        DcMotorEx launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        launcherMotors.setDirection(DcMotorEx.Direction.REVERSE);

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1443.28, 1443.28, 927.968, 566.107)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamSM"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .build();

        // Reverse the right side motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(5, 1.5, 0, 12.5);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double distanceToTag = 0;
            double calculatedPower = launcherPower;

            // AprilTag detection
            if (!aprilTag.getDetections().isEmpty()) {
                AprilTagDetection tag = aprilTag.getDetections().get(0);
                distanceToTag = tag.ftcPose.range;

                // Jika auto-adjust enabled, hitung power berdasarkan jarak
                if (autoAdjustEnabled) {
                    calculatedPower = calculateLauncherPower(distanceToTag);
                }

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance (inch)", "%.2f", distanceToTag);
            } else {
                telemetry.addLine("No AprilTag detected");
            }

            // TankDrive Movement
            double y = gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x;

//            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);

            double leftPower = Range.clip(x - y, -1.0, 1.0);
            double rightPower = Range.clip(x + y, -1.0, 1.0);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Intake control (hold button)
            if (gamepad1.right_bumper) {
                intakeMotors.setPower(-1);
            }
            else if(gamepad1.left_bumper) {
                intakeMotors.setPower(1);
            }
            else {
                intakeMotors.setPower(0);
            }

            // Toggle Auto-Adjust dengan tombol X
            if (gamepad1.x && !xButtonPressed) {
                autoAdjustEnabled = !autoAdjustEnabled;
                xButtonPressed = true;
            } else if (!gamepad1.x) {
                xButtonPressed = false;
            }

            // Launcher power adjustment with D-pad (hanya jika auto-adjust OFF)
            if (!autoAdjustEnabled) {
                if (gamepad1.dpad_up && !dpadUpPressed) {
                    launcherPower += powerIncrement;
                    if (launcherPower > 3000) launcherPower = 3000;
                    dpadUpPressed = true;
                } else if (!gamepad1.dpad_up) {
                    dpadUpPressed = false;
                }

                if (gamepad1.dpad_down && !dpadDownPressed) {
                    launcherPower -= powerIncrement;
                    if (launcherPower < 0) launcherPower = 0;
                    dpadDownPressed = true;
                } else if (!gamepad1.dpad_down) {
                    dpadDownPressed = false;
                }
            }

            // TOGGLE LAUNCHER - Left Bumper untuk ON/OFF
            if (gamepad1.right_trigger == 1.0 && !leftBumperPressed) {
                launcherOn = !launcherOn;
                leftBumperPressed = true;
            }
            else if(!gamepad1.y) {

            }
//            else if ((!gamepad1.y)) {
//                launcherOn = !launcherOn;
//                launcherMotors.setDirection(DcMotorSimple.Direction.FORWARD);
//                leftBumperPressed = true;
//            }
            else {
                leftBumperPressed = false;
            }

            // Set launcher velocity berdasarkan toggle status
            if (launcherOn) {
                if (autoAdjustEnabled) {
                    launcherMotors.setVelocity(calculatedPower);
                } else {
                    launcherMotors.setVelocity(launcherPower);
                }
            } else {
                launcherMotors.setVelocity(0);
            }

            // Telemetry
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("Left stick: Move forward/backward");
            telemetry.addLine("Right stick: Rotate");
            telemetry.addLine("Right/Left Bumper: Intake (hold)");
            telemetry.addLine("Right/Left Trigger: Toggle launcher ON/OFF");
            telemetry.addLine("X button: Toggle Auto-Adjust Power");
            telemetry.addLine("D-pad UP/DOWN: Manual power adjust");
            telemetry.addLine("A button: Pusher");
            telemetry.addLine("");
            telemetry.addData("Launcher Status", launcherOn ? "ON" : "OFF");
            telemetry.addData("Auto-Adjust", autoAdjustEnabled ? "ENABLED" : "DISABLED");

            if (autoAdjustEnabled) {
                telemetry.addData("Calculated Power", "%.0f", calculatedPower);
            } else {
                telemetry.addData("Manual Power Setting", "%.0f", launcherPower);
            }

            telemetry.addData("Current Velocity", "%.2f", launcherMotors.getVelocity());
            telemetry.update();
        }
    }

    /**
     * Hitung launcher power berdasarkan jarak menggunakan fungsi polynomial
     * y = -0.0000459204x^4 + 0.0137484x^3 - 1.36047x^2 + 56.68778x + 585.16626
     *
     * @param distance Jarak ke AprilTag dalam inch
     * @return Power yang diperlukan untuk launcher
     */
    private double calculateLauncherPower(double distance) {
        double x = distance;

        // Koefisien polynomial
//        double a4 = -0.0000459204;
//        double a3 = 0.0137484;
//        double a2 = -1.36047;
        double a1 = 1.00314;
        double a0 = 1258.83539;

        // Hitung polynomial: y=1258.83539\cdot1.00314^{x}
        double power =
//                a4 * Math.pow(x, 4)
//                + a3 * Math.pow(x, 3)
//                + a2 * Math.pow(x, 2)

                + a0 * Math.pow(a1, x);

        // Clamp power dalam range yang aman
        if (power < 0) power = 0;
        if (power > 3000) power = 3000;

        return power;
    }
}