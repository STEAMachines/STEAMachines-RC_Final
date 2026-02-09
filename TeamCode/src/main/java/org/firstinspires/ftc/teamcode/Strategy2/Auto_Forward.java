package org.firstinspires.ftc.teamcode.Strategy2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.STEAMachines_bot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auto_Forward extends LinearOpMode {
    @Override
    public void runOpMode() {
        STEAMachines_bot drive = new STEAMachines_bot(hardwareMap);

        waitForStart();
        TrajectorySequence trj = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(120)
                .build();

        if(opModeIsActive()) {
            drive.followTrajectorySequence(trj);
        }
    }
}
