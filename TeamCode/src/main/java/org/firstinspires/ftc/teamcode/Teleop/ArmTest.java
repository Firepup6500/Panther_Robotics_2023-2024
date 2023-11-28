package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ArmTest")
public class ArmTest extends OpMode {
    DcMotor ArmLift;

    @Override
    public void init(){
        ArmLift = hardwareMap.get(DcMotor.class,"arm_lift");
        telemetry.addData("ftc", "first");
        telemetry.addData("Init", "is a success");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            ArmLift.setPower(-5);
        }
        if (gamepad2.left_bumper) {
            ArmLift.setPower(5);
        }
        ArmLift.setPower(0);
    }
}
