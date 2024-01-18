package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class distanceLogic extends LinearOpMode {
    DistanceSensor distance;

    public int averageDistance = 0;
    public int margin = 30;
    public double currentDistance = 0;
    public int countPixels = 0;
    public boolean pixelPassing = false;

    @Override
    public void runOpMode() throws InterruptedException {

        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            currentDistance = distance.getDistance(DistanceUnit.MM);

            if(currentDistance < averageDistance - margin && pixelPassing == false){
                countPixels++;
                pixelPassing = true;
            }
            if((averageDistance + 5) > currentDistance && currentDistance > (averageDistance - 5) && pixelPassing == true){
                pixelPassing = false;
            }

        telemetry.addData("Current pixels: ", pixelPassing);
        telemetry.addData("Current Distance: ", currentDistance);
        telemetry.addData("Pixel Passing? ", pixelPassing);


        }

    }
}
