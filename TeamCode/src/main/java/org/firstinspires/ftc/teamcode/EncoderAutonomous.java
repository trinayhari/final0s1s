package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class EncoderAutonomous extends EncoderTest2{

    @Override
    public void runOpMode() {


        encoderDrive(DRIVE_SPEED, 10, 10, 1);


    }



}
