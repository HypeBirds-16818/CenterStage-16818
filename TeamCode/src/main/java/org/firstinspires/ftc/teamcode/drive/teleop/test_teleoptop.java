package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "testtop")
public class test_teleoptop extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.008, i = 0, d = 0.0001;
    public static double f = 0.1;

    public static int target = 0;

    private final double ticks_in_degree = 537.7;

    private DcMotorEx top_motor_1, top_motor_2;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double poder = 0;
            if (gamepad1.right_bumper)
            {
                poder = 1;
            }
            if(gamepad1.left_bumper)
            {
                poder = -1;
            }
            drive.setElevadorPower(poder);
            telemetry.addData("Posicion",drive.getElevadorPos());
            telemetry.update();

        }


    }

}
