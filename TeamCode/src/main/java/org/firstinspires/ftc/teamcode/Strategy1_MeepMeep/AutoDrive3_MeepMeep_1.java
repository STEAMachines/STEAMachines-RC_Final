package org.firstinspires.ftc.teamcode.Strategy1_MeepMeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.STEAMachines_bot;

@Disabled
@Autonomous(name="Autonomous3_DECODE_MeepMeep(Test)", group="STEAMachines_DECODE")
public class AutoDrive3_MeepMeep_1 extends LinearOpMode {
    DcMotor shooterMotors;
    DcMotor intakeMotors;
    CRServo handleServo;
    @Override
    public void runOpMode() {
        STEAMachines_bot drive = new STEAMachines_bot(hardwareMap);
        shooterMotors = hardwareMap.get(DcMotor.class, "launcherMotors");
        intakeMotors = hardwareMap.get(DcMotor.class, "intakeMotors");
        handleServo = hardwareMap.get(CRServo.class, "handleServo");
        Pose2d startPose = new Pose2d(-48, 48, -Math.PI/4);
        drive.setPoseEstimate(startPose);
        waitForStart();
    }
}
