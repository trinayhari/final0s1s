
package org.firstinspires.ftc.teamcode ;


import android.media.MediaPlayer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MechanumDrive;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.man.c.SkystoneDetector.Location.One;
import static org.man.c.SkystoneDetector.Location.Two;
import static org.man.c.SkystoneDetector.Location.Zero;


@Autonomous(name="SkystoneAutoMode", group="Auto")

public class SkystoneAutoMode extends MechanumDrive {

    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor shooter = null;

    public Servo claw = null;
    public DcMotor arm = null;
    public DistanceSensor x = null;
    public DistanceSensor side = null;
    public DistanceSensor front =null;
public DcMotor shaft = null;
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        x = hardwareMap.get(DistanceSensor.class, "distance");
        side = hardwareMap.get(DistanceSensor.class, "side");
        front = hardwareMap.get(DistanceSensor.class, "front");
        Rev2mDistanceSensor sensor1Time = (Rev2mDistanceSensor) side;

        Rev2mDistanceSensor sensorTime = (Rev2mDistanceSensor) x;
        Rev2mDistanceSensor sensorFTime = (Rev2mDistanceSensor) front;


        double sp = 0;
        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.get(DcMotor.class, "ARM");

        shaft = hardwareMap.get(DcMotor.class, "ST");

        leftDrive = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter = hardwareMap.get(DcMotor.class, "SHOOT");


        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        org.man.c.SkystoneDetector detector = new org.man.c.SkystoneDetector(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }
                }
        );

        telemetry.addData("RangeFront", x.getDistance(DistanceUnit.INCH));

        telemetry.update();


        // Send telemetry message to indicate successful Encoder reset
       /*
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        */


        waitForStart();
        claw.setPosition(0.0);
        sleep(1000);
        phoneCam.closeCameraDevice();

        if (detector.getLocation() == One) {
            while(front.getDistance(DistanceUnit.INCH)>21){
                arm.setPower(0.2);
                claw.setPosition(0.1);
                leftBackDrive.setPower(0.3);
                rightDrive.setPower(0.3);
                rightBackDrive.setPower(0.3);
                leftDrive.setPower(0.3);
            }
            arm.setPower(-0.2);
            sleep(1000);
            claw.setPosition(0.5);
            arm.setPower(0.2);
            sleep(1000);
            while(front.getDistance(DistanceUnit.INCH)<43){
                leftBackDrive.setPower(-0.3);
                rightDrive.setPower(-0.3);
                rightBackDrive.setPower(-0.3);
                leftDrive.setPower(-0.3);
            }
            while (side.getDistance(DistanceUnit.INCH) <= 35){
                leftDrive.setPower(-.3);
                leftBackDrive.setPower(-.3);
                rightBackDrive.setPower(.3);

                rightDrive.setPower(.3);
            }
            shooter.setPower(1.0);
            sleep(3000);
            shooter.setPower(1.0);
            shaft.setPower(0.6);
            sleep(4000);

        }




        if (detector.getLocation() == Zero) {

            telemetry.addData("RangeFront", x.getDistance(DistanceUnit.INCH));

            telemetry.update();
            while (x.getDistance(DistanceUnit.INCH) <= 45) {

                telemetry.addData("RangeFront", x.getDistance(DistanceUnit.INCH));

                telemetry.update();
                arm.setPower(0.2);
                claw.setPosition(0.1);
                leftBackDrive.setPower(0.3);
                rightDrive.setPower(0.3);
                rightBackDrive.setPower(0.3);
                leftDrive.setPower(0.3);

            }
            arm.setPower(-0.2);
            sleep(100);
            claw.setPosition(0.5);
            arm.setPower(0.5);
            sleep(1000);

        }




        if (detector.getLocation() == Two) {
            sleep(5000);
            while (x.getDistance(DistanceUnit.INCH) <= 45) {

                telemetry.addData("RangeFront", x.getDistance(DistanceUnit.INCH));

                telemetry.update();
                arm.setPower(0.2);
                claw.setPosition(0.1);
                leftBackDrive.setPower(0.3);

                rightBackDrive.setPower(0.3);
                leftDrive.setPower(0.3);
                rightDrive.setPower(0.3);

            }
            while (side.getDistance(DistanceUnit.INCH) <= 32) {
                leftDrive.setPower(-.3);
                leftBackDrive.setPower(-.3);
                rightBackDrive.setPower(.3);

                rightDrive.setPower(.3);
            }

            leftBackDrive.setPower(0.3);
            rightDrive.setPower(0.3);
            rightBackDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            sleep(1000);
            arm.setPower(-0.2);
            sleep(2000);
            claw.setPosition(0.5);
            arm.setPower(0.2);


        }


    }
}


