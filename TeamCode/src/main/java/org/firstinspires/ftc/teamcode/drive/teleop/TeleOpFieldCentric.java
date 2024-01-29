package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@TeleOp(name = "TeleOpCCM")
public class TeleOpFieldCentric extends LinearOpMode {
    public static int target = 0;

    public enum OuttakeState{
        BASE,
        UP,
        DROP
    };

    OuttakeState outtakeState = OuttakeState.BASE;
    //public static boolean launchPlane = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(SavePose.currentPose);

        drive.setSlideMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        drive.setServoBase(.26);
        drive.setServoCaja(.3);


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Movimiento de chasis

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

            //Small movement
            if(gamepad1.dpad_left)
                drive.setMotorPowers(-.2, .2, -.2, .2);
            if(gamepad1.dpad_right)
                drive.setMotorPowers(.2, -.2, .2, -.2);
            if(gamepad1.dpad_up)
                drive.setMotorPowers(.2, .2, .2, .2);
            if(gamepad1.dpad_down)
                drive.setMotorPowers(-.2, -.2, -.2, -.2);



            if(gamepad1.right_bumper){
                drive.setServoAvion(0.75);
            }

            //Elevador
            drive.getPID(target);
            if(gamepad2.a){
                //base
                target = 0;
            }
            if(gamepad2.b){
                //segunda linea
                target = 1800;
            }
            if(gamepad2.y){
                //tercera linea
                target = 2500;
            }
            if(gamepad2.right_bumper)
                target += 100;
            if(gamepad2.left_bumper)
                target -= 100;

            //Intake
            double intakePower = Range.clip(gamepad2.right_stick_y, -1, 1);
            drive.setIntakePower(intakePower);

            //Outake
            //Poner en posicion de subida
            if(gamepad2.dpad_up) {
                drive.setServoCaja(.3);
                drive.setServoBase(.26);
            }
            //Poner en posicion de bajada, faltan valores reales
            if(gamepad2.dpad_down) {
                drive.setServoBase(.35);
                drive.setServoCaja(.4);
            }
            if(gamepad2.dpad_left)
            {
                drive.setServoCaja(.4);
            }
            if(gamepad2.dpad_right)
            {
                drive.setServoBase(.35);
            }


            //Climber (to do)

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