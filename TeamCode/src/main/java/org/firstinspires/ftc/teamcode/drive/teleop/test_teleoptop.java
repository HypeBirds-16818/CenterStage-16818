package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "testtop")
public class test_teleoptop extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.009, i = 0, d = 0.0001;
    public static double f = 0.14;

    public static int target = 0;

    private final double ticks_in_degree = 751.8;

    private DcMotorEx top_motor_1;
    private DcMotorEx top_motor_2;

    @Override
    public void runOpMode() throws InterruptedException {


        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        top_motor_1 = hardwareMap.get(DcMotorEx.class, "top_motor_1");
        top_motor_2 = hardwareMap.get(DcMotorEx.class, "top_motor_2");
        top_motor_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        top_motor_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            controller.setPID(p, i, d);
            int basePos = (top_motor_1.getCurrentPosition() + top_motor_2.getCurrentPosition()) / 2;
            double pid = controller.calculate(basePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            top_motor_1.setPower(power);
            top_motor_2.setPower(power);

            telemetry.addData("base position: ", basePos);
            telemetry.addData("target position: ", target);
            telemetry.update();

        }
    }

}
