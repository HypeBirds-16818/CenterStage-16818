package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleOpCCM")
public class TeleOpFieldCentric extends LinearOpMode {
    private DcMotorEx intakeMotor, linearSlide;

    private PIDController controller;

    public static double p = 0.008, i = 0, d = 0.0001;
    public static double f = 0.1;
    public static int target = 0;
    private final double ticks_in_degree = 537.7;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        controller = new PIDController(p, i, d);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PoseStorage.currentPose);

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        linearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            controller.setPID(p, i, d);
            int basePos = linearSlide.getCurrentPosition();
            double pid = controller.calculate(basePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            linearSlide.setPower(power);

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepad2.a){
                //base
                target = 0;
            }
            if(gamepad2.b){
                //segunda linea
                target = 2300;
            }
            if(gamepad2.x){
                //tercera linea
                target = 3000;
            }

            if(gamepad2.left_bumper == true){
                //intake adentro
                intakeMotor.setPower(1);
            }

            if(gamepad2.right_bumper == true){
                //intake afuera
                intakeMotor.setPower(-1);
            }

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}