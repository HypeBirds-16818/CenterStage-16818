package org.firstinspires.ftc.teamcode.autonomo;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BlueElementHP;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Azul HP")
public class AzulHumanPlayer extends LinearOpMode {
    private OpenCvCamera camera;
    private BlueElementHP blueElementHP;
    private String webcamName = "Webcam 1";
    Pose2d start_pose = new Pose2d(15.92, -67.31, Math.toRadians(90.00));

    @Override
    public void runOpMode() throws InterruptedException {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        blueElementHP = new BlueElementHP();
        camera.setPipeline(blueElementHP);

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
                .lineTo(new Vector2d(16, -43.06))
                .turn(Math.toRadians(90))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .build();

        TrajectorySequence Azul_Medio = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(10.92, -38.06))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .back(3)
                .build();

        TrajectorySequence Azul_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .splineTo(new Vector2d(16.81, -48.67), Math.toRadians(87.52))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .build();
//        TrajectorySequence forw = drive.trajectorySequenceBuilder(start_pose)


        while (!isStarted()) {
            telemetry.addData("ROTATION: ", blueElementHP.getAnalysis());
            telemetry.update();
        }
        waitForStart();
        drive.setServoAutonomo(.07);
        if (isStopRequested()) return;
        if (blueElementHP.getAnalysis()==1) { drive.followTrajectorySequence(Azul_Medio); }
        else if(blueElementHP.getAnalysis()==2) { drive.followTrajectorySequence(Azul_Derecha); }
        else { drive.followTrajectorySequence(Azul_Izquierda); }

    }

}
