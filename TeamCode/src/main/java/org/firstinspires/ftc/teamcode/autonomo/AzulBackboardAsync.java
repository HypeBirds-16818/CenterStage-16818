package org.firstinspires.ftc.teamcode.autonomo;


import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BlueElement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name = "Azul Back")
public class AzulBackboardAsync extends LinearOpMode {
    private OpenCvCamera camera;
    private BlueElement blueElement;
    private String webcamName = "Webcam 1";
    static int x;
    int target = 0;
    enum State {
        TRAJECTORY_1,
        ELEVADOR_SUBIR,
        DEJAR_SERVO,
        WAIT_1,
        ELEVADOR_BAJAR,
        WAIT_2,
        ESTACIONAR,
        IDLE            // Our bot will enter the IDLE state when done
    }
    State currentState = State.IDLE;
    Pose2d start_pose = new Pose2d(12.19, 67.31, Math.toRadians(270.00));
    public void runOpMode() throws InterruptedException {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        blueElement = new BlueElement();
        camera.setPipeline(blueElement);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start_pose);
        double waitTime1 = 3;
        ElapsedTime waitTimer1 = new ElapsedTime();

        TrajectorySequence Azul_Izquierda = drive.trajectorySequenceBuilder(start_pose)
                .splineTo(new Vector2d(21.81, 53.67), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.5)) // Lower servo
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .setReversed(true)
                .splineTo(new Vector2d(49.3 , 46.3), Math.toRadians(0))
                .build();

        TrajectorySequence Azul_Medio = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(11.25, 38.06))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.5)) // Lower servo
                .waitSeconds(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .strafeTo(new Vector2d(46.31, 36))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(49,44))
                .build();

        TrajectorySequence Azul_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(12.15, 44.06))
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.5)) // Lower servo
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .setReversed(true)
                .splineTo(new Vector2d(49, 31.5), Math.toRadians(0))
                .build();



        x = blueElement.getAnalysis();
        while (!isStarted()) {
            telemetry.addData("Posicion: ", blueElement.getAnalysis());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;
        if (blueElement.getAnalysis()==1) { drive.followTrajectorySequenceAsync(Azul_Medio); }
        else if(blueElement.getAnalysis()==2) { drive.followTrajectorySequenceAsync(Azul_Derecha); }
        else { drive.followTrajectorySequenceAsync(Azul_Izquierda); }

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    drive.setServoBase(.26);
                    drive.setServoCaja(.3);
                    if (!drive.isBusy()) {
                        currentState = State.ELEVADOR_SUBIR;
                    }
                    break;
                case ELEVADOR_SUBIR:
                    target =1600;
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = AzulBackboardAsync.State.DEJAR_SERVO;

                    }
                    break;
                case DEJAR_SERVO:
                    drive.setServoBase(.35);
                    drive.setServoCaja(.4);

                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_2:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.ELEVADOR_BAJAR;

                    }
                    break;
                case ELEVADOR_BAJAR:
                    drive.setServoBase(.26);
                    target = 0;
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.getPID(target);
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();


            SavePose.currentPose = drive.getPoseEstimate();
        }
    }

}
