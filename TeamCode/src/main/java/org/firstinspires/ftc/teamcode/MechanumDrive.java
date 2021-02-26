package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.app.Application;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Bundle;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;

@TeleOp(name = "MechanumDrive", group = "LinearOpmode")

public class MechanumDrive extends EncoderTest2 {
    private ElapsedTime runtime = new ElapsedTime();
    public MediaPlayer player1 = new MediaPlayer();
    private DistanceSensor disSense;
    private DistanceSensor x1;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor intake = null;
    public DcMotor shooter = null;
    private boolean isAOn = false;
    private double slowModeValue = 1.0;
    public DcMotor shaft = null;
    public DcMotor arm = null;
    private DistanceSensor x;
    public Servo claw = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        x = hardwareMap.get(DistanceSensor.class, "distance");
        Rev2mDistanceSensor sensorTime = (Rev2mDistanceSensor) x;
        player1 = MediaPlayer.create(hardwareMap.appContext, R.raw.song);

        //   x1 = hardwareMap.get(DistanceSensor.class, "distance1");
        //  Rev2mDistanceSensor sensorTime = (Rev2mDistanceSensor)x;
        //   Rev2mDistanceSensor sensor1Time = (Rev2mDistanceSensor)x1;
        telemetry.addData(">>", "Start");
        telemetry.update();


        //leftDrive = front left motor
        //rightDrive = right front motor
        //leftBackDrive = back left motor
        //rightBackDrive = back right motor
        //armJoint = arm motor
        //sweeper = sweeper servo
        //latch = rack and pinion motor
        //boxJoint = box moving servo


        leftDrive = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "IT");
        shaft = hardwareMap.get(DcMotor.class, "ST");
        arm = hardwareMap.get(DcMotor.class, "ARM");
        shooter = hardwareMap.get(DcMotor.class, "SHOOT");

        claw = hardwareMap.servo.get("claw");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            telemetry.addData("RangeFront", x.getDistance(DistanceUnit.INCH));

            telemetry.update();

            double servoPosition = 0.0;


            telemetry.update();


/*

          while (gamepad1.a) {
              disSense.getDistance(DistanceUnit.INCH);
              telemetry.update();
              if (disSense.getDistance(DistanceUnit.INCH) >= 38) {


                  leftDrive.setPower(-.3);
                  leftBackDrive.setPower(-.3);
                    rightBackDrive.setPower(-.3);

                    rightDrive.setPower(.3);
                  telemetry.update();
                }

                else if (disSense.getDistance(DistanceUnit.INCH) <= 38) {
                    leftDrive.setPower(.3);
                    leftBackDrive.setPower(.3);
                    rightBackDrive.setPower(.3);

                    rightDrive.setPower(-.3);
                  telemetry.update();
                }

            }
*/

            double armPower = 0.0;
            double shooterPower = 0.0;
            if (gamepad1.b ) {
                shooterPower= 1.0;
            } else {
                shooterPower= 0.0;
            }
            if (gamepad1.right_trigger >= 0.1) {
                armPower = gamepad1.right_trigger;

            }
            else if  (gamepad1.left_trigger >= 0.1) {
                armPower = -1*(gamepad1.left_trigger);
            }

            arm.setPower(armPower);


            if(gamepad1.y) {
                claw.setPosition(0.1);
                telemetry.addData("CLaw pos",claw.getPosition());
                telemetry.update();

            } else {
                claw.setPosition(0.75);
                telemetry.addData("CLaw pos",claw.getPosition());
                telemetry.update();
            }
            if (gamepad1.right_bumper) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }
            if (gamepad1.left_bumper) {
                shaft.setPower(0.8);
            } else {
                shaft.setPower(0.0);
            }
            shooter.setPower(shooterPower);

/*
            while (gamepad1.x) {

                if (Math.abs(x.getDistance(DistanceUnit.INCH) - x1.getDistance(DistanceUnit.INCH)) <= 3) {
                    telemetry.addData("Alligned:", "Yes");

                } else if ((x.getDistance(DistanceUnit.INCH) >= x1.getDistance(DistanceUnit.INCH))) {
                    leftDrive.setPower(1.0);
                    leftBackDrive.setPower(1.0);
                } else {
                    rightDrive.setPower(1.0);
                    rightBackDrive.setPower(1.0);
                }


            }


          if (gamepad1.x){
                servoPosition = 0.5;
            }
            if(gamepad1.y){
                servoPosition= 1.0;
            }
            else{
                servoPosition=0.0;
            }

         */

            //Driving
            double leftstickx = 0;
            double leftsticky = 0;
            double rightstickx = 0;
            double wheelpower;
            double stickangleradians;
            double rightX;
            double leftfrontpower;
            double rightfrontpower;
            double leftrearpower;
            double rightrearpower;
            double dpadpower = .2;
            double dpadturningpower = .4;
            double speedmodifier = 1;

            if (gamepad2.right_bumper) {
                speedmodifier = .5;
            }
            if (gamepad2.left_bumper) {
                speedmodifier = 1;
            }

            if (gamepad2.dpad_up) {
                leftsticky = dpadpower;
            } else if (gamepad2.dpad_right) {
                leftstickx = dpadturningpower;
            } else if (gamepad2.dpad_down) {
                leftsticky = -dpadpower;
            } else if (gamepad2.dpad_left) {
                leftstickx = -dpadturningpower;
            } else {
                leftstickx = gamepad2.left_stick_x * speedmodifier;
                leftsticky = -gamepad2.left_stick_y * speedmodifier;
                rightstickx = gamepad2.right_stick_x * speedmodifier;
            }
            if (Math.abs(leftsticky) <= .15) {
                leftsticky = 0;
            }
            wheelpower = Math.hypot(leftstickx, leftsticky);
            stickangleradians = Math.atan2(leftsticky, leftstickx);

            stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

            rightX = rightstickx * .5;
            leftfrontpower = wheelpower * Math.cos(stickangleradians) + rightX;
            rightfrontpower = wheelpower * Math.sin(stickangleradians) - rightX;
            leftrearpower = wheelpower * Math.sin(stickangleradians) + rightX;
            rightrearpower = wheelpower * Math.cos(stickangleradians) - rightX;

            leftDrive.setPower(leftfrontpower);
            rightDrive.setPower(rightfrontpower);
            leftBackDrive.setPower(leftrearpower);
            rightBackDrive.setPower(rightrearpower);



            telemetry.update();


        }

    }


}
