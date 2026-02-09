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
@TeleOp(name="PIDF_Launchers", group="STEAMachines_DECODE")
public class PIDF_Launchers extends LinearOpMode {
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    public static double target = 2000;
    public static double low = 1000;
    public static double error = 0;
    public static int toggle = 0;
    public static boolean hilo = true;
    public static boolean intake = false;
    public static boolean shooter = false;

    DcMotorEx launcherMotors;
    DcMotor intakeMotors;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        launcherMotors = hardwareMap.get(DcMotorEx.class, "launcherMotors");
        intakeMotors = hardwareMap.get(DcMotor.class, "intakeMotors");
        launcherMotors.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidf = launcherMotors.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while(opModeIsActive()) {
                if (intake) {
                    intakeMotors.setPower(-1);
                }
                else {
                    intakeMotors.setPower(0);
                }
                launcherMotors.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//            telemetry.addData("P:", pidf.p);
//            telemetry.addData("I:", pidf.i);
//            telemetry.addData("D:", pidf.d);
//            telemetry.addData("F:", pidf.f);
                if (shooter) {
                    launcherMotors.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
                    if (hilo) {
                        launcherMotors.setVelocity(target);}
                    else {
                        launcherMotors.setVelocity(low);
                    }
                } else {
                    launcherMotors.setVelocity(0);
                }
                telemetry.addData("Target", target);
                telemetry.addData("Shooter:", launcherMotors.getVelocity());
                error = target - launcherMotors.getVelocity();
            telemetry.addData("Error", error);
                telemetry.update();
                sleep(100);
            }
        }
    }
