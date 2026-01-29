package org.firstinspires.ftc.teamcode.Strategy1_MeepMeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.STEAMachines_bot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoDrive1-UsingTest",group="STEAMachines_DECODE")
public class AutoDrive1_MeepMeep_test extends LinearOpMode {
    DcMotor intakeMotors;
    DcMotorEx launcherMotors;
    Servo handleServo;
    @Override
    public void runOpMode() {
        STEAMachines_bot drive = new STEAMachines_bot(hardwareMap);
        intakeMotors  = hardwareMap.get(DcMotor.class, "intakeMotors");
        launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");
        handleServo   = hardwareMap.get(Servo.class, "handleServo");
        launcherMotors.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        launcherMotors.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,3,0,12.65));

        TrajectorySequence trj = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(10)
                .turn(Math.toRadians(25))
                .waitSeconds(3)
                .addDisplacementMarker(30, ()-> {
                    launcherMotors.setVelocity(1850);
                })
                .waitSeconds(10)
                .addTemporalMarker(3.5, ()-> {
                    intakeMotors.setPower(-1);
                })
                .build();
//        TrajectorySequence trj = drive.trajectorySequenceBuilder(startPose)
////                .forward(10)
////                .turn(Math.toRadians(45))
//                .forward(150)
//                .waitSeconds(3)
//                .turn(Math.toRadians(-90))
////                .addTemporalMarker(0.5, () -> {
////                    launcherMotors.setPower(1);
////                })
////                .addTemporalMarker(() -> {
////                    launcherMotors.setPower(0);
////                })
//                .waitSeconds(3)
//                .turn(Math.toRadians(45))
//                .forward(50)
//                .forward(20)
////                .addTemporalMarker(0.5, () -> {
////                    intakeMotors.setPower(1.0);
////                })
////                .addTemporalMarker(() -> {
////                    intakeMotors.setPower(0);
////                })
//                .turn(Math.toRadians(-90))
//                .forward(25)
//                .back(25)
//                .build();

//        TrajectorySequence trj_turn_1 = drive.trajectorySequenceBuilder(startPose)
//                .turn(Math.toRadians(45))
//                .build();
//        TrajectorySequence trj_turn_2 = drive.trajectorySequenceBuilder(startPose)
//                .turn(Math.toRadians(-90))
//                .build();
//        TrajectorySequence trj_turn_3 = drive.trajectorySequenceBuilder(startPose)
//                .turn(Math.toRadians(90))
//                .build();
//        Trajectory trj_drive = drive.trajectoryBuilder(startPose)
//                .forward(50)
//                .build();
//        Trajectory trj_drive_1 = drive.trajectoryBuilder(startPose)
//                .forward(25)
//                .build();
//        Trajectory trj_back = drive.trajectoryBuilder(startPose)
//                .back(25)
//                .build();
//        TrajectorySequence trj_wait = drive.trajectorySequenceBuilder(startPose)
//                .waitSeconds(3)
//                .build();
//        TrajectorySequence mainTrajectory = drive.trajectorySequenceBuilder(startPose)
//                .forward(20)
//                .addTemporalMarker(0.5, () -> {
//                    intakeMotors.setPower(1.0);
//                })
//                .addTemporalMarker(() -> {
//                    intakeMotors.setPower(0);
//                })
//                .build();

        telemetry.addData("Status", "Initialized - Tunggu Start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            drive.followTrajectorySequence(trj);
//            drive.followTrajectory(trj_drive);
//            drive.followTrajectorySequence(trj_turn_1);
//            drive.followTrajectorySequence(trj_wait);
//            drive.followTrajectorySequence(trj_turn_1);
//            drive.followTrajectory(trj_drive);
//            drive.followTrajectorySequence(mainTrajectory);
//            drive.followTrajectorySequence(trj_turn_2);
//            drive.followTrajectory(trj_drive_1);
//            drive.followTrajectory(trj_back);

//            drive.followTrajectorySequence(trj_turn_1);
//            drive.followTrajectory(trj_drive);
//            drive.followTrajectorySequence(trj_wait);
//            drive.followTrajectorySequence(trj_turn_1);
//            drive.followTrajectory(trj_drive);
//            drive.followTrajectorySequence(mainTrajectory);
//            drive.followTrajectorySequence(trj_turn_2);
//            drive.followTrajectory(trj_drive_1);
//            drive.followTrajectory(trj_back);
//            drive.followTrajectorySequence(trj);

            intakeMotors.setPower(0);
            launcherMotors.setPower(0);
            handleServo.setPosition(0);
        }
    }
}
