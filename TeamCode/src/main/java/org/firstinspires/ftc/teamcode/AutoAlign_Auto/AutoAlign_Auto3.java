package org.firstinspires.ftc.teamcode.AutoAlign_Auto;

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
public class AutoAlign_Auto3 extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public static double launcherPower = 1550;

    boolean launcherOn = false;
    boolean autoAdjustEnabled = false;

    enum StatesAuto {
        AUTO
    }

    enum StatesTele {
        TELE
    }

    @Override
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

        PIDFCoefficients pidfCoefficients_short = new PIDFCoefficients(100, 0.5, 0, 12.3);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients_short);

        waitForStart();
            double distanceToTag = 0;
            double calculatedPower = launcherPower;

            if (!aprilTag.getDetections().isEmpty()) {
                AprilTagDetection tag = aprilTag.getDetections().get(0);
                distanceToTag = tag.ftcPose.range;

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance (inch)", "%.2f", distanceToTag);
            } else {
                telemetry.addLine("No AprilTag detected");
            }

            double finalPower = autoAdjustEnabled ? calculateLauncherPower(distanceToTag) : launcherPower;

        TrajectorySequence trj = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(15))
                .forward(115)
                .addTemporalMarker(0.1, () -> {
                    launcherMotors.setVelocity(finalPower);
                })
                .waitSeconds(3)
                .addTemporalMarker(5, () -> {
                    intakeMotors.setPower(-1);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(5.5, () -> {
                    intakeMotors.setPower(0);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(6, () -> {
                    intakeMotors.setPower(-1);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(6.5, () -> {
                    intakeMotors.setPower(0);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(7, () -> {
                    intakeMotors.setPower(-1);
                })
//                .splineTo(new Vector2d(-40, 0), -265)
//                .waitSeconds(0.25)
//                .addTemporalMarker(7.5, ()-> {
//                    intakeMotors.setPower(0);
//                })
                .build();
        if(opModeIsActive()) {
            drive.followTrajectorySequence(trj);

            intakeMotors.setPower(0);
        }
    }

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
