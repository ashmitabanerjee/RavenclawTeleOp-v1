package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@TeleOp
@Config
public class RavenclawTeleOp extends LinearOpMode {

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;

    private Servo leftLinkageServo;
    private Servo rightLinkageServo;
    double leftLinkageServoPosition = 0.6;
    double rightLinkageServoPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightLinkageServo = hardwareMap.get(Servo.class, "rightLinkageServo");
        leftLinkageServo = hardwareMap.get(Servo.class, "leftLinkageServo");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLinkageServo.setPosition(rightLinkageServoPosition);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);
        }
    }
}
