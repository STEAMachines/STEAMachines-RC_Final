package org.firstinspires.ftc.teamcode.Strategy1_MeepMeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.STEAMachines_bot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoDrive3-UsingTest",group="STEAMachines_DECODE")
public class AutoDrive3_MeepMeep_test extends LinearOpMode {
    DcMotor intakeMotors;
    DcMotorEx launcherMotors;
    Servo handleServo;
//    double target = 1580;
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
                .forward(135)
                .addTemporalMarker(0.1, () -> {
                    launcherMotors.setVelocity(1550);
                })
                .waitSeconds(3)
                .back(20)
                .waitSeconds(0.45)
                .back(5)
                .addDisplacementMarker(155, ()-> {
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
}
