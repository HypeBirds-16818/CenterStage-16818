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
    enum Intake {
        TAKING,
        RAISING1,
        RAISING2,
        TURNING,
        OPENING,
        TURNINGBACK,
        FALLING1,
        FALLING2
    }

    StateMachine machine = new StateMachineBuilder(){
        .state(Intake.TAKING)
        .build();
    }

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 0; //Añadir ticks

    private DcMotorEx bottom_motor_1;
    private DcMotorEx bottom_motor_2;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bottom_motor_1 = hardwareMap.get(DcMotorEx.class, "bottom_motor_1");
        bottom_motor_2 = hardwareMap.get(DcMotorEx.class, "bottom_motor_2");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            controller.setPID(p, i, d);
            int basePos = (bottom_motor_1.getCurrentPosition() + bottom_motor_2.getCurrentPosition())/2;
            double pid = controller.calculate(basePos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

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
