package org.firstinspires.ftc.teamcode.drive.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Integrated Test")
public class test_integration extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    CRServo planeServo;


    @Override
    public void runOpMode() throws InterruptedException {
        drive.setModeIntake(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        planeServo = hardwareMap.get(CRServo.class, "planeServo");
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            setPid(drive, "Top", 100);

            if(gamepad1.a){
                planeServo.setPower(1);
            }
            if(gamepad1.b){
                planeServo.setPower(-1);
            }
            if(gamepad1.x){
                planeServo.setPower(0);
            }
        }
    }

    public void setPid(SampleMecanumDrive drive, String PIDcase, int target){
        if (PIDcase.equals("Top")){
            drive.getTopPID(target);
        }
        else if (PIDcase.equals("Bottom")){
            drive.getBottomPID(target);
        }



    }

}


