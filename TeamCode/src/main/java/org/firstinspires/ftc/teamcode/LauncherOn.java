package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class LauncherOn extends LinearOpMode {
    public void runOpMode() {
        DcMotorEx launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");
        waitForStart();
        while(opModeIsActive()) {
            launcherMotors.setVelocity(1600);
        }
    }
}
