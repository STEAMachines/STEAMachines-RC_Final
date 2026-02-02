package org.firstinspires.ftc.teamcode.Strategy1_MeepMeep;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.STEAMachines_bot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="AutoDrive3-UsingTest",group="STEAMachines_DECODE")
public class AutoDrive3_MeepMeep_test extends LinearOpMode {
    DcMotor intakeMotors;
    DcMotorEx launcherMotors;
    Servo handleServo;
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    AprilTagDetection tag;
//    double target = 1580;

//    aprilTag = new AprilTagProcessor.Builder()
//            .setLensIntrinsics(1443.28, 1443.28, 927.968, 566.107)
//                .build();
//
//    visionPortal = new VisionPortal.Builder()
//            .setCamera(hardwareMap.get(WebcamName.class, "WebcamSM"))
//            .setCameraResolution(new Size(1280, 720))
//            .addProcessor(aprilTag)
//                .build();
    @Override
    public void runOpMode() {
        STEAMachines_bot drive = new STEAMachines_bot(hardwareMap);
        intakeMotors  = hardwareMap.get(DcMotor.class, "intakeMotors");
        launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");
        handleServo   = hardwareMap.get(Servo.class, "handleServo");
        launcherMotors.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        launcherMotors.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100,0.5,0,12.3));

        TrajectorySequence trj = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(115)
                .addTemporalMarker(0.1, () -> {
                    launcherMotors.setVelocity(1558);
                })
                .waitSeconds(3)
//                .back(20)
//                .waitSeconds(0.45)
//                .back(5)
                .addDisplacementMarker(110, ()-> {
                    intakeMotors.setPower(-1);
                })
//                .addDisplacementMarker(155, ()-> {
//                    if (launcherMotors.getVelocity() < target + 10 && launcherMotors.getVelocity() > target - 10) {
//                        return;
//                    }
//                })
                .waitSeconds(15)
//                .splineTo(new Vector2d(75, -40), Math.toRadians(-180))
//                .waitSeconds(3)
//                .forward(5)
//                .addDisplacementMarker(160, ()-> {
//                    intakeMotors.setPower(-1);
//                })
                .build();

//        TrajectorySequence trj = drive.trajectorySequenceBuilder(startPose)
//                .forward(25)
//                .turn(Math.toRadians(180))
//                .addTemporalMarker(3, () -> {
//                    launcherMotors.setPower(1.0);
//                })
//                .addTemporalMarker(() -> {
//                    launcherMotors.setPower(0);
//                })
//                .waitSeconds(3)
//                .turn(Math.toRadians(90))
//                .forward(25)
//                .addTemporalMarker(3, () -> {
//                    intakeMotors.setPower(1.0);
//                })
//                .addTemporalMarker(() -> {
//                    intakeMotors.setPower(0);
//                })
////                .waitSeconds(3)
//                .back(25)
//                .turn(Math.toRadians(-90))
//                .addTemporalMarker(3, () -> {
//                    launcherMotors.setPower(1.0);
//                })
//                .addTemporalMarker(() -> {
//                    launcherMotors.setPower(0);
//                })
//                .turn(Math.toRadians(110))
//                .forward(50)
//                .addTemporalMarker(3, () -> {
//                    intakeMotors.setPower(1.0);
//                })
//                .addTemporalMarker(() -> {
//                    intakeMotors.setPower(0);
//                })
//                .back(50)
//                .turn(Math.toRadians(-110))
//                .addTemporalMarker(3, () -> {
//                    launcherMotors.setPower(1.0);
//                })
//                .addTemporalMarker(() -> {
//                    launcherMotors.setPower(0);
//                })
//                .build();
//        TrajectorySequence intakeMove = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(3, () -> {
//                    intakeMotors.setPower(1.0);
//                })
//                .addTemporalMarker(() -> {
//                    intakeMotors.setPower(0);
//                })
//                .build();
//        TrajectorySequence launcherMove = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(3, () -> {
//                    launcherMotors.setPower(1.0);
//                })
//                .addTemporalMarker(() -> {
//                      launcherMotors.setPower(0);
//                })
//                .build();

        telemetry.addData("Status", "Initialized - Tunggu Start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            drive.followTrajectorySequence(trj);
//            drive.followTrajectory(trj_drive);
//            drive.followTrajectorySequence(trj_wait);
//            drive.followTrajectory(trj_drive_1);
//            drive.followTrajectorySequence(trj_turn);
//            drive.followTrajectory(trj_back);
//            drive.followTrajectorySequence(trj_wait);

            intakeMotors.setPower(0);
            launcherMotors.setPower(0);
            handleServo.setPosition(0);
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
