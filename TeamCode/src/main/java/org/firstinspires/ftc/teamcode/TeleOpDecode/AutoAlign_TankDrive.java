package org.firstinspires.ftc.teamcode.TeleOpDecode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name="AutoAlign_TankDrive", group="STEAMachines_DECODE")
public class AutoAlign_TankDrive extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    double speedMultiplier = 2500;
    public static double launcherPower = 1550; // Default Power for Launchers (Access through FTC-Dash)
    double powerIncrement = 50; // Amount of the power that changes

    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;

    // Toggle launcher variables
    boolean launcherOn = false;
    boolean leftBumperPressed = false;

    // Auto-adjust launcher power based on the distances.
    boolean autoAdjustEnabled = false;
    boolean xButtonPressed = false;

    //Changing between mode-shooter_range
    boolean range = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotorEx leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        DcMotorEx rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        DcMotor intakeMotors = hardwareMap.get(DcMotor.class, "intakeMotors");
        DcMotorEx launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");

//        leftDrive.

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        launcherMotors.setDirection(DcMotorEx.Direction.REVERSE);

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1443.28, 1443.28, 927.968, 566.107)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamSM"))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        // Reverse the right side motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //PIDF SET
        PIDFCoefficients pidfCoefficients_short = new PIDFCoefficients(100, 0.5, 0, 12.3);
        PIDFCoefficients pidfCoefficients_long = new PIDFCoefficients(100, 0, 0, 12.5);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients_short);

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
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x * 1.1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);

            double leftPower = (y + x)  / denominator * speedMultiplier;
            double rightPower = (y - x) / denominator * speedMultiplier;

            leftDrive.setVelocity(leftPower);
            rightDrive.setVelocity(rightPower);

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

            // Toggle Auto-Adjust using X-Buttons
            if (gamepad1.x && !xButtonPressed) {
                autoAdjustEnabled = !autoAdjustEnabled;
                xButtonPressed = true;
            } else if (!gamepad1.x) {
                xButtonPressed = false;
            }

            // Launcher power adjustment with D-pad (only if the auto-adjust OFF)
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
            if (gamepad1.bWasPressed() && !leftBumperPressed) {
                launcherOn = !launcherOn;
                leftBumperPressed = false;
            } else if (gamepad1.yWasPressed()) {
                leftBumperPressed = false;
            } else if (gamepad1.right_trigger > 0.2) {
                leftBumperPressed = true;
                launcherOn = false;
            }

            // Set launcher velocity berdasarkan toggle status
            if (launcherOn) {
                if (autoAdjustEnabled) {
                    launcherMotors.setVelocity(calculatedPower);
                } else {
                    launcherMotors.setVelocity(launcherPower);
                }
            } else if (leftBumperPressed) {
                launcherMotors.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                launcherMotors.setVelocity(0);
            }
            else {
                launcherMotors.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                launcherMotors.setVelocity(0);
            }

            //MODE: SHORT_RANGE & LONG_RANGE

            // Telemetry
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("Left stick: Move forward/backward");
            telemetry.addLine("Right stick: Rotate");
            telemetry.addLine("Right/Left Bumper: Intake (hold)");
            telemetry.addLine("Right/Left Trigger: Toggle launcher ON/OFF");
            telemetry.addLine("X button: Toggle Auto-Adjust Power");
            telemetry.addLine("D-pad UP/DOWN: Manual power adjust");
            telemetry.addLine("");
            telemetry.addData("Launcher Status", launcherOn ? "ON" : "OFF");
            telemetry.addData("Auto-Adjust", autoAdjustEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("MODE: ", range ? "LONG-RANGE" : "SHORT-RANGE");

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