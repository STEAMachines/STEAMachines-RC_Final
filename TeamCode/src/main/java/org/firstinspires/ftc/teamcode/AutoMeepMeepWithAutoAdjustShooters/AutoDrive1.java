package org.firstinspires.ftc.teamcode.AutoMeepMeepWithAutoAdjustShooters;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LauncherOn;
import org.firstinspires.ftc.teamcode.drive.STEAMachines_bot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous
public class AutoDrive1 extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public static double launcherPowers = 1550;
    boolean launcherOn = false;
    boolean autoAdjustEnabled = false;

    public void runOpMode() {
        STEAMachines_bot drive = new STEAMachines_bot(hardwareMap);
        DcMotorEx intakeMotors = hardwareMap.get(DcMotorEx.class, "intakeMotors");
        DcMotorEx launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");

        launcherMotors.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotors.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotors.setDirection(DcMotor.Direction.REVERSE);
        intakeMotors.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,2.5,0,12.5));

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1443.28, 1443.28, 927.968, 566.107)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamSM"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .build();

        TrajectorySequence trj = drive.trajectorySequenceBuilder(new Pose2d())
                .back(10)
                .turn(Math.toRadians(35))
                .addTemporalMarker(0.1, ()-> {
                    launcherOn = !launcherOn;
                })
                .waitSeconds(10)
                .addTemporalMarker(5, ()-> {
                    intakeMotors.setVelocity(2500);
                })
                .build();

        waitForStart();
        if(opModeIsActive()) {
            double distancetoTag = 0;
            double calculatedPower = launcherPowers;

            if(!aprilTag.getDetections().isEmpty()) {
                AprilTagDetection tag = aprilTag.getDetections().get(0);
                distancetoTag = tag.ftcPose.range;

                if(autoAdjustEnabled) {
                    calculatedPower = calculatedPowerLaunchers(distancetoTag);
                }
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance (inch)", "%.2f", distancetoTag);
            }
            else {
                telemetry.addLine("No AprilTag detected");
            }
            if(launcherOn) {
                if(autoAdjustEnabled) {
                    launcherMotors.setVelocity(calculatedPower);
                }
                else {
                    launcherMotors.setVelocity(launcherPowers);
                }
            }
            else {
                launcherMotors.setVelocity(0);
            }

            drive.followTrajectorySequence(trj);

            intakeMotors.setVelocity(0);
            launcherMotors.setVelocity(0);
        }
    }

    public double calculatedPowerLaunchers(double distance) {
        double x = distance;

        //Koefisien polynomial
        double a1 = 1.00314;
        double a0 = 1258.83539;

        double power = a0 * Math.pow(a1, x);

        if (power < 0) power = 0;
        if (power < 3000) power = 3000;

        return power;
    }
}
