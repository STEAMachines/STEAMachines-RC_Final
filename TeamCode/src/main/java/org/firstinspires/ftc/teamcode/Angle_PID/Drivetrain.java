//package org.firstinspires.ftc.teamcode.Angle_PID;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//
//public class Drivetrain {
//    public DcMotor leftDrive;
//    public DcMotor rightDrive;
//
//    HardwareMap hwMap;
//
//    public void init(HardwareMap ahwMap) {
//
//        /**
//         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
//         * **/
//        hwMap = ahwMap;
//
//        /**
//         * Hardware initialized and String Names are in the Configuration File for Hardware Map
//         * **/
//
//        // Control HUb
//        leftDrive = hwMap.get(DcMotor.class, "leftDrive");
//        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
//
//
//        /**
//         * Allow the 4 wheel motors to be run without encoders since we are doing a time based autonomous
//         * **/
//        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        /**
//         *Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards
//         * **/
//
//        //THIS IS THE CORRECT ORIENTATION
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        /**
//         * Reverses shooter motor to shoot the correct way and same with the conveyor motor
//         * **/
//
//        /**
//         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
//         * **/
//        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        /**
//         *The 4 mecanum wheel motors, intake, conveyor, and shooter motor/servo are set to 0 power to keep it from moving when the user presses the INIT button
//         * **/
//        leftDrive.setPower(0);
//        rightDrive.setPower(0);
//
//    }
//
//    public void power(double output){
//        leftDrive.setPower(-output);
//        rightDrive.setPower(-output);
//    }
//    public void moveRobot(double leftStickY, double rightStickX){
//        /**
//         * Wheel powers calculated using gamepad 1's inputs leftStickY, leftStickX, and rightStickX
//         * **/
//        double leftPower = leftStickY + rightStickX;
//        double rightPower = leftStickY - rightStickX;;
//
//        /**
//         * Sets the wheel's power
//         * **/
//        leftDrive.setPower(leftPower);
//        rightDrive.setPower(rightPower);
//    }
//}