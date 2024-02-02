package org.firstinspires.ftc.teamcode.autonomo;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.vision.RedElement;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Rojo HP")
public class RojoHumanPlayer extends LinearOpMode {
        private OpenCvCamera camera;
        private RedElement redElement;
        private String webcamName = "Webcam 1";
    Pose2d start_pose = new Pose2d(-36.61, -67.31, Math.toRadians(90.00));

        @Override
        public void runOpMode() throws InterruptedException {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
            redElement = new RedElement();
            camera.setPipeline(redElement);

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
            TrajectorySequence Rojo_Izquierda = drive.trajectorySequenceBuilder(start_pose)
                    .lineTo(new Vector2d(-36.61, -44.06))
                    .turn(Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.3)) // Lower servo
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                    .waitSeconds(.01)
                    .build();


            TrajectorySequence Rojo_Medio = drive.trajectorySequenceBuilder(start_pose)
                    .lineTo(new Vector2d(-36.61, -44.06))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.3)) // Lower servo
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                    .waitSeconds(.01)
                    .build();

            TrajectorySequence Rojo_Derecha = drive.trajectorySequenceBuilder(start_pose)
                    .lineTo(new Vector2d(-36.61, -44.06))
                    .turn(Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.3)) // Lower servo
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                    .waitSeconds(.01)
                    .build();

            int x = redElement.getAnalysis();
            while (!isStarted()) {
                telemetry.addData("ROTATION: ", redElement.getAnalysis());
                telemetry.update();
            }


            waitForStart();
            if (isStopRequested()) return;

            if (redElement.getAnalysis()==1) { drive.followTrajectorySequence(Rojo_Medio); }
            else if(redElement.getAnalysis()==2) { drive.followTrajectorySequence(Rojo_Derecha); }
            else { drive.followTrajectorySequence(Rojo_Izquierda); }
    }

}
