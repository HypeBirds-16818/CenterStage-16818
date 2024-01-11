package org.firstinspires.ftc.teamcode.drive.autonomo;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AzulBackboard extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence Azul_Backboard_Medio = drive.trajectorySequenceBuilder(new Pose2d(12.09, 65.34, Math.toRadians(90.00)))
                .setReversed(true)
                .lineTo(new Vector2d(11.74, 32.58))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(3,() ->
                        drive.setIntakePower(-.2))
                .splineTo(new Vector2d(47.47, 36.44), Math.toRadians(0.00))
                .setReversed(true)
                .build();

        TrajectorySequence Azul_Backboard_Izquierda = drive.trajectorySequenceBuilder(new Pose2d(12.09, 65.34, Math.toRadians(90.00)))
                .setReversed(true)
                .lineTo(new Vector2d(13.49, 32.58))
                .UNSTABLE_addTemporalMarkerOffset(3,() ->
                        drive.setIntakePower(-.2))
                .splineTo(new Vector2d(31.71, 15.24), Math.toRadians(15.89))
                .setReversed(false)
                .splineTo(new Vector2d(47.47, 36.96), Math.toRadians(5.50))
                .setReversed(true)
                .build();


        TrajectorySequence Azul_Backboard_Derecha = drive.trajectorySequenceBuilder(new Pose2d(12.09, 65.34, Math.toRadians(90.00)))
                .setReversed(true)
                .lineTo(new Vector2d(10.51, 32.76))
                .UNSTABLE_addTemporalMarkerOffset(3,() ->
                        drive.setIntakePower(-.2))
                .splineTo(new Vector2d(47.47, 36.96), Math.toRadians(5.86))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(Azul_Backboard_Medio);
    }
}
