package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class PIDF_Intake extends LinearOpMode {

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    public static double error = 0;
    public static double target = 2500;

    public void runOpMode() {
        DcMotorEx intakeMotors = hardwareMap.get(DcMotorEx.class, "intakeMotors");
        DcMotorEx launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcherMotors.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotors.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = intakeMotors.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients_short = new PIDFCoefficients(100, 0.5, 0, 12.3);
        launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients_short);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                intakeMotors.setVelocity(target);
//                launcherMotors.setVelocity(1550);
            }
            else if(!gamepad1.a) {
                intakeMotors.setVelocity(0);
//                launcherMotors.setVelocity(0);
            }

//            telemetry.addData("P", pidf.p);
//            telemetry.addData("I", pidf.i);
//            telemetry.addData("D", pidf.d);
//            telemetry.addData("F", pidf.f);

            intakeMotors.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
            telemetry.addData("Target", target);
            telemetry.addData("Intake", intakeMotors.getVelocity());
            error = target - intakeMotors.getVelocity();
            telemetry.addData("Error", error);
            telemetry.update();
            sleep(100);

        }
    }
}
