package org.firstinspires.ftc.teamcode.Strategy1_MeepMeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.STEAMachines_bot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous1_DECODE_MeepMeep(Test)", group="STEAMachines_DECODE")
@Disabled
public class AutoDrive1_MeepMeep_1 extends LinearOpMode {
    DcMotor intakeMotors;
    DcMotor shooterMotors;
    Servo handleServo;

    @Override
    public void runOpMode() {
        STEAMachines_bot drive = new STEAMachines_bot(hardwareMap);
        intakeMotors = hardwareMap.get(DcMotor.class, "intakeMotors");
        shooterMotors = hardwareMap.get(DcMotor.class, "launcherMotors");
        handleServo = hardwareMap.get(Servo.class, "handleServo");
        Pose2d startPose = new Pose2d(60, 15, Math.PI);
        drive.setPoseEstimate(startPose);
        waitForStart();
        TrajectorySequence trj_turn_1 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(45))
                .build();
        TrajectorySequence trj_turn_2 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence trj_turn_3 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .build();
        Trajectory trj_drive = drive.trajectoryBuilder(startPose)
                .forward(50)
                .build();
        Trajectory trj_drive_1 = drive.trajectoryBuilder(startPose)
                .forward(25)
                .build();
        Trajectory trj_back = drive.trajectoryBuilder(startPose)
                .back(25)
                .build();
        TrajectorySequence trj_wait = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(3)
                .build();
        Trajectory trj_shooter = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(3, () -> {
                    shooterMotors.setPower(-1);
                })
                .build();
        TrajectorySequence trj_intake = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .addTemporalMarker(3, () -> {
                    intakeMotors.setPower(1);
                })
                .forward(10)
                .build();
        if (opModeIsActive()) {
            drive.followTrajectorySequence(trj_intake);
        }
//        drive.followTrajectorySequence(trj_turn_1);
//        drive.followTrajectory(trj_drive);
//        drive.followTrajectorySequence(trj_wait);
//        drive.followTrajectorySequence(trj_turn_1);
//        drive.followTrajectory(trj_drive);
//        drive.followTrajectory(trj_shooter);
//        drive.followTrajectorySequence(trj_turn_2);
//        drive.followTrajectory(trj_drive_1);
//        drive.followTrajectorySequence(trj_intake);
//        drive.followTrajectory(trj_back);
//        drive.followTrajectorySequence(trj_turn_3);
    }


}
