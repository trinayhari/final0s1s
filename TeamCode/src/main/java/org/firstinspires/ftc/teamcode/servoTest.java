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
@TeleOp(name = "servoTest", group = "LinearOpmode")
public class servoTest extends LinearOpMode {


    public Servo claw=null;
    private DcMotor arm;

   double servoPos = 0.0;
    @Override
    public void runOpMode() {

        arm = hardwareMap.get(DcMotor.class,"ARM");
        claw = hardwareMap.servo.get("claw");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();


        while (opModeIsActive()) {

        double armPower;
            arm.setPower(0.0);
          if(gamepad1.right_trigger>=0){
              arm.setPower(gamepad1.right_trigger);
          }
            if(gamepad1.left_trigger>=0){
                arm.setPower(gamepad1.left_trigger);
            }

            if(gamepad1.a){
                claw.setPosition(0.0);
            }
         else{
                claw.setPosition(0.5);

            }

            arm.setPower(0.0);



        }


    }




}
