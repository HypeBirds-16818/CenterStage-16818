package org.firstinspires.ftc.teamcode.drive.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class test_integration extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



    @Override
    public void runOpMode() throws InterruptedException {
        setPid(drive, "Top", 100);
    }

    public void setPid(SampleMecanumDrive drive, String PIDcase, int target){
        if (PIDcase == "Top"){
            drive.getTopPID(target);
        }
        else if (PIDcase == "Bottom"){
            drive.getBottomPID(target);
        }



    }

}


