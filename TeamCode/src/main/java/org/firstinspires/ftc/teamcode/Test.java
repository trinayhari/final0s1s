
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test" , group = "TeleOp")
public class Test extends OpMode{
    DcMotor test;

    @Override
    public void init() {
        test = hardwareMap.dcMotor.get("te");
    }
    @Override
    public void loop() {
        double x = 0.0;

        if (gamepad1.a) {
            x = 1.0;
        }
        test.setPower(x);


    }


}





