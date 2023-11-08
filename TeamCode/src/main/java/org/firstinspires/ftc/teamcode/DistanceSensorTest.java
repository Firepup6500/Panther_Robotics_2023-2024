package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name = "Distance Sensor Test", group = "Tests")
public class DistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor leftSensor = hardwareMap.get(DistanceSensor.class, "left");
        //DistanceSensor centerSensor = hardwareMap.get(DistanceSensor.class, "center");
        DistanceSensor rightSensor = hardwareMap.get(DistanceSensor.class, "right");
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();
        resetRuntime();
        double left;
        double right;
        int direction;
        while (opModeIsActive()) {
            left = leftSensor.getDistance(DistanceUnit.MM);
            right = rightSensor.getDistance(DistanceUnit.MM);
            telemetry.addData("Runtime", getRuntime());
            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            if (left-right >= 200) {
                direction = 1;
                telemetry.addData("Direction", "Right");
            } else if (right-left >= 200) {
                direction = -1;
                telemetry.addData("Direction", "Left");
            } else {
                direction = 0;
                telemetry.addData("Direction", "Forward");
            }
            telemetry.update();
        }
    }

    private boolean buttonPreviousState;

    public boolean buttonClick(boolean button) {
        boolean returnVal;
        returnVal = button && !buttonPreviousState;
        buttonPreviousState = button;
        return returnVal;
    }
}
