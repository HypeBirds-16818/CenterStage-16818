package org.firstinspires.ftc.teamcode.drive.autonomo;

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

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name = "Azul_async")
public class AzulBackboardAsync extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private static final String TFOD_MODEL_ASSET = "ModeloSoloPajaro.tflite";

    private static final String[] LABELS = {
            "PAJARO"
    };
    int target = 0;
    enum State {
        TRAJECTORY_1,
        ELEVADOR_SUBIR,
        DEJAR_SERVO,
        WAIT_1,
        ELEVADOR_BAJAR,
        ESTACIONAR,
        IDLE            // Our bot will enter the IDLE state when done
    }
    State currentState = State.IDLE;
    Pose2d start_pose = new Pose2d(12.09, 61.31, Math.toRadians(270.00));
    public void runOpMode() throws InterruptedException {

        initTfod();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(start_pose);
        double waitTime1 = 3;
        ElapsedTime waitTimer1 = new ElapsedTime();

        TrajectorySequence Azul_Izquierda = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(23.26, 48.56))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .setReversed(true)
                .lineTo(new Vector2d(45.40, 40.93))
                .build();

        TrajectorySequence Azul_Medio = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(12.96, 25.01))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .strafeTo(new Vector2d(49.40, 35.47))
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence Azul_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(11.35, 33.49))
                .turn(90)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .setReversed(true)
                .lineTo(new Vector2d(45.21, 28.47))
                .build();

        TrajectorySequence Azul_Final = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(46, -60))
                .setReversed(true)
                .lineTo(new Vector2d(61.88, -61.12))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        double x = 0;
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
        }

        currentState = State.TRAJECTORY_1;
        if (x < 640) { drive.followTrajectorySequenceAsync(Azul_Izquierda); }
        else if(x > 1280) { drive.followTrajectorySequenceAsync(Azul_Derecha); }
        else { drive.followTrajectorySequenceAsync(Azul_Medio); }

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
                    target =700;
                    if (!drive.isBusy()) {
                        currentState = State.DEJAR_SERVO;
                    }
                    break;

                case DEJAR_SERVO:
                    drive.setServoBase(.35);
                    drive.setServoCaja(.4);

                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.ELEVADOR_BAJAR;

                    }
                    break;
                case ELEVADOR_BAJAR:
                    target = 0;
                    if (!drive.isBusy()) {
                        currentState = State.ESTACIONAR;
                    }
                    break;
                case ESTACIONAR:
                    drive.followTrajectorySequenceAsync(Azul_Final);
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
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));
        tfod.setZoom(2);

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
}
