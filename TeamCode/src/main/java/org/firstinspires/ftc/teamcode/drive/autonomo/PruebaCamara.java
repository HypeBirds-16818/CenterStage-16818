package org.firstinspires.ftc.teamcode.drive.autonomo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "PruebaCamara")
public class PruebaCamara extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private static final String TFOD_MODEL_ASSET = "ModeloSoloPajaro.tflite";

    private static final String[] LABELS = {
            "PAJARO"
    };
    int target = 0;

    enum State {
        ESPERA_MEDIO,
        ENMEDIO,
        ESPERA_IZQ,
        IZQUIERDA,
        ESPERA_DER,
        DERECHA,
        TRAYECTORIA,
        ELEVADOR_SUBIR,
        DEJAR_SERVO,
        WAIT_1,
        ELEVADOR_BAJAR,
        ESTACIONAR,
        IDLE
    }

    State currentState = State.IDLE;
    Pose2d start_pose = new Pose2d(12.19, -67.31, Math.toRadians(90.00));

    public void runOpMode() throws InterruptedException {
        initTfod();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start_pose);

        double waitTime1 = 3;
        ElapsedTime waitTimer1 = new ElapsedTime();

        TrajectorySequence Enmedio = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(11.25, -44.06))
                .build();

        TrajectorySequence Izquierda = drive.trajectorySequenceBuilder(Enmedio.end())
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence Derecha = drive.trajectorySequenceBuilder(Enmedio.end())
                .turn(Math.toRadians(-180))
                .build();

        TrajectorySequence Rojo_Izquierda = drive.trajectorySequenceBuilder(start_pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .setReversed(true)
                .splineTo(new Vector2d(49.30, -29.40), Math.toRadians(-13.24))
                .build();
//
//
        TrajectorySequence Rojo_Medio = drive.trajectorySequenceBuilder(start_pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .strafeTo(new Vector2d(46.31, -36.00))
                .turn(Math.toRadians(-90))
                .build();
//
        TrajectorySequence Rojo_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .setReversed(true)
                .lineTo(new Vector2d(35, -42.23))
                .build();
//
//
        TrajectorySequence Rojo_Final = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(46, -60))
                .setReversed(true)
                .lineTo(new Vector2d(61.88, -61.12))
                .build();


        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequenceAsync(Enmedio);
        currentState = State.ESPERA_MEDIO;
//        if (x < 640) { drive.followTrajectorySequenceAsync(Rojo_Izquierda); }
//        else if(x > 1280) { drive.followTrajectorySequenceAsync(Rojo_Derecha); }
//        else { drive.followTrajectorySequenceAsync(Rojo_Medio); }


        drive.setServoBase(.26);
        drive.setServoCaja(.3);
        waitTimer1.reset();

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            double x;
            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
            }
            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case ESPERA_MEDIO:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.ENMEDIO;
                    }
                case ENMEDIO:
                    if (!drive.isBusy() && currentRecognitions.size()>0) {
                        currentState = State.TRAYECTORIA;
                        drive.followTrajectorySequence(Rojo_Medio);
                    }
                    if (!drive.isBusy() && currentRecognitions.size()==0){
                        currentState = State.ESPERA_IZQ;
                        drive.followTrajectorySequence(Izquierda);
                        waitTimer1.reset();
                    }
                    break;
                case ESPERA_IZQ:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.IZQUIERDA;

                    }
                case IZQUIERDA:
                    if (!drive.isBusy() && currentRecognitions.size()>0) {
                        currentState = State.TRAYECTORIA;
                        drive.followTrajectorySequence(Rojo_Izquierda);
                    }
                    if (!drive.isBusy() && currentRecognitions.size()==0){
                        currentState = State.ESPERA_DER;
                        drive.followTrajectorySequence(Derecha);
                        waitTimer1.reset();
                    }
                case ESPERA_DER:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.DERECHA;

                    }
                case DERECHA:
                    if(!drive.isBusy()){
                        currentState = State.TRAYECTORIA;
                        drive.followTrajectorySequence(Rojo_Derecha);
                    }
                case TRAYECTORIA:
                    if(!drive.isBusy()){
                        currentState = State.ELEVADOR_SUBIR;
                    }
                case ELEVADOR_SUBIR:
                    target = 600;
                    if (!drive.isBusy()) {
                        currentState = State.DEJAR_SERVO;
                    }
                    break;

                case DEJAR_SERVO:
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
                        currentState = State.IDLE;
                    }
                    break;
                case ESTACIONAR:
                    drive.followTrajectorySequenceAsync(Rojo_Final);
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
