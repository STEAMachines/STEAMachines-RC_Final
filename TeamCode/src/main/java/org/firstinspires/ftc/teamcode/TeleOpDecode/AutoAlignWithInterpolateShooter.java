package org.firstinspires.ftc.teamcode.TeleOpDecode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Config
//@TeleOp
@Disabled
public class AutoAlignWithInterpolateShooter extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    double speedMultiplier = 0.7;
    public static double launcherPower = 1500; //Default power for launchers.
    double powerIncrement = 100; //The amount of power change.

    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;

    //Toggle launcher variables
    boolean launcherOn = false;
    boolean rightTriggerPressed = false;

    //Toggle AutoAdjust based on launcherPowers.
    boolean autoAdjustEnabled = false;
    boolean xButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        DcMotorEx launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");
        DcMotor intakeMotors = hardwareMap.get(DcMotor.class, "intakeMotors");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        launcherMotors.setDirection(DcMotorSimple.Direction.REVERSE);

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1443.28, 1443.28, 927.968, 566.107)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamSM" ))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .build();

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10, 1.5, 0, 12.5);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            double distanceToTag = 0;
            double finalPower = launcherPower;

            // AprilTag detection
            if (!aprilTag.getDetections().isEmpty()) {
                AprilTagDetection tag = aprilTag.getDetections().get(0);
                distanceToTag = tag.ftcPose.range;

                // Jika auto-adjust enabled, hitung power berdasarkan jarak
                if (autoAdjustEnabled) {
                    finalPower = calculateLauncherPower(distanceToTag);
                }

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance (inch)", "%.2f", distanceToTag);
            } else {
                telemetry.addLine("No AprilTag detected");
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x * 1.1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);

            double leftPower = (y + x)  / denominator * speedMultiplier;
            double rightPower = (y - x) / denominator * speedMultiplier;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Intake control (hold button)
            if (gamepad1.right_bumper) {
                intakeMotors.setPower(-1);
            }
            else if (gamepad1.left_bumper) {
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

            // TOGGLE LAUNCHER - RightTrigger untuk ON/LeftTrigger untuk OFF
            if (gamepad1.right_trigger == 1.0 && !rightTriggerPressed) {
                launcherOn = !launcherOn;
                rightTriggerPressed = true;
            } else if (!(gamepad1.left_trigger == 1.0)) {
                rightTriggerPressed = false;
            }

            // Set launcher velocity berdasarkan toggle status
            if (launcherOn) {
                launcherMotors.setVelocity(finalPower);
            } else {
                launcherMotors.setVelocity(0);
            }

            // Telemetry
            telemetry.addLine("=== STEAMachines-CONTROLS ===");
            telemetry.addLine("Left stick: Move forward/backward/strafe");
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
                telemetry.addData("Power (from distance)", "%.0f", finalPower);
            } else {
                telemetry.addData("Manual Power Setting", "%.0f", launcherPower);
            }

            telemetry.addData("Current Velocity", "%.2f", launcherMotors.getVelocity());
            telemetry.update();
        }
    }
    private double calculateLauncherPower(double distance) {
        double[][] dataPoints = {
                {27.26, 1220},
                {33.24, 1240},
                {39.34, 1240},
                {48.91, 1240},
                {55.22, 1320},
                {59.69, 1360},
                {62.63, 1360},
                {65.54, 1360},
                {69.57, 1360},
                {72.85, 1400},
                {76.92, 1440},
                {80.53, 1480},
                {113.02, 1620},
                {117.47, 1840},
                {127.86, 1820},
                {132.33, 1840},
        };

        // Jika jarak di bawah data minimum, gunakan power minimum
        if (distance <= dataPoints[0][0]) {
            return dataPoints[0][1];
        }

        // Jika jarak di atas data maksimum, gunakan power maksimum
        if (distance >= dataPoints[dataPoints.length - 1][0]) {
            return dataPoints[dataPoints.length - 1][1];
        }

        // Cari dua titik terdekat untuk interpolasi linear
        for (int i = 0; i < dataPoints.length - 1; i++) {
            double dist1 = dataPoints[i][0];
            double power1 = dataPoints[i][1];
            double dist2 = dataPoints[i + 1][0];
            double power2 = dataPoints[i + 1][1];

            // Jika distance berada di antara dua titik ini
            if (distance >= dist1 && distance <= dist2) {
                // Interpolasi linear: y = y1 + (y2 - y1) * (x - x1) / (x2 - x1)
                double power = power1 + (power2 - power1) * (distance - dist1) / (dist2 - dist1);

                // Clamp power dalam range yang aman
                if (power < 0) power = 0;
                if (power > 3000) power = 3000;

                return power;
            }
        }

        // Fallback (seharusnya tidak pernah sampai sini)
        return 1500;
    }
}
