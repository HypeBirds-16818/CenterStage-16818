package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@Config
@TeleOp(name = "test")
public class test_teleop extends LinearOpMode {
    public enum intakeState  {
        IDLE,
        TAKING,
        RAISING1,
        RAISING2,
        TURNING,
        OPENING,
        TURNINGBACK,
        FALLING1,
        FALLING2
    }

    public int stage = 0;
    private PIDController controller;

    public static double p = 0.03, i = 0, d = 0.0001;
    public static double f = 0.1;

    public static int target = 0;

    private final double ticks_in_degree = 1425.1;

    private DcMotorEx bottom_motor_1;
    private DcMotorEx bottom_motor_2;

    @Override
    public void runOpMode() throws InterruptedException {


        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bottom_motor_1 = hardwareMap.get(DcMotorEx.class, "bottom_motor_1");
        bottom_motor_2 = hardwareMap.get(DcMotorEx.class, "bottom_motor_2");
        bottom_motor_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bottom_motor_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            controller.setPID(p, i, d);
            int basePos = (bottom_motor_1.getCurrentPosition() + bottom_motor_2.getCurrentPosition()) / 2;
            double pid = controller.calculate(basePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            bottom_motor_1.setPower(power);
            bottom_motor_2.setPower(power);

            telemetry.addData("base position: ", basePos);
            telemetry.addData("target position: ", target);
            telemetry.update();

        }
    }
}

//https://state-factory.gitbook.io/state-factory/essential-usage
