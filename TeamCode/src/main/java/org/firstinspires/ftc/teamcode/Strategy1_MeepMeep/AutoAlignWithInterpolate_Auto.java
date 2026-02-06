package org.firstinspires.ftc.teamcode.Strategy1_MeepMeep;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.STEAMachines_bot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous
public class AutoAlignWithInterpolate_Auto extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public static double launcherPower = 1550;

    boolean launcherOn = false;
    boolean autoAdjustEnabled = false;

    public void runOpMode() throws InterruptedException{
        STEAMachines_bot drive = new STEAMachines_bot(hardwareMap);
        DcMotor intakeMotors = hardwareMap.get(DcMotor.class, "intakeMotors");
        DcMotorEx launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");

        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        launcherMotors.setDirection(DcMotorSimple.Direction.REVERSE);

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1443.28, 1443.28, 927.968, 566.107)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamSM"))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        PIDFCoefficients pidfCoefficients_long = new PIDFCoefficients(100, 0, 0, 12.5);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients_long);

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

        if(launcherOn) {
            if(autoAdjustEnabled) {
                launcherMotors.setVelocity(calculatedPower);
            }
            else {
                launcherMotors.setVelocity(launcherPower);
            }
        }
        else {
            launcherMotors.setVelocity(0);
        }

        waitForStart();

        if(opModeIsActive()) {

            TrajectorySequence trj = drive.trajectorySequenceBuilder(new Pose2d())
//                    .back(10)
//                    .turn(Math.toRadians(-35))
                    .addTemporalMarker(0.1, ()-> {
                        launcherOn = !launcherOn;
                    })
                    .waitSeconds(10)
//                    .addTemporalMarker(5, ()-> {
//                        intakeMotors.setPower(-1);
//                    })
                    .build();

            drive.followTrajectorySequence(trj);

            intakeMotors.setPower(0);
//            launcherMotors.setVelocity(0);

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
