package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "turret")
public class turret extends OpMode {
    double Kp = 0.015;
    double Ki = 0.005;
    double Kd = 22.0;
    //tuned with maxPower = 0.4
    enum TuningMode { P, I, D, MAX_POWER, DEAD_ZONE }
    TuningMode selectedMode = TuningMode.P;

    double step = 0.001;

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx turretMotor;
    Limelight3A limelight;

    double maxPower = 0.4;
    double deadZone = 0.5;

    boolean autoAimStatus = false;

    boolean lastUp, lastDown, lastLeft, lastRight;
    boolean lastA, lastB, lastX, lastY, lastLB;
    boolean lastRB;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        timer.reset();
        telemetry.addLine("initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        tuningInput();
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = 0;
            timer.reset();
            telemetry.addLine("no tag detected");
            telemetryDataDisplay(null, "searching");
            return;
        }

        double dt = timer.seconds();
        timer.reset();
        
        if (dt <= 0) dt = 0.001;

        double error = result.getTx();

        if (!autoAimStatus) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = 0;
        } else {
            if (Math.abs(error) < deadZone) {
                turretMotor.setPower(0);
                integralSum = 0;
            } else {
                integralSum += error * dt;
                integralSum = Range.clip(integralSum, -10, 10);

                double derivative = (error - lastError) / dt;
                lastError = error;

                double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                output = Range.clip(output, -maxPower, maxPower);
                turretMotor.setPower(output);
            }
        }

        telemetryDataDisplay(error, "running");
    }

    void tuningInput() {
        if (gamepad1.right_bumper && !lastRB) {
            autoAimStatus = !autoAimStatus;
        }
        lastRB = gamepad1.right_bumper;

        if (gamepad1.a && !lastA) selectedMode = TuningMode.P;
        if (gamepad1.b && !lastB) selectedMode = TuningMode.I;
        if (gamepad1.x && !lastX) selectedMode = TuningMode.D;
        if (gamepad1.y && !lastY) selectedMode = TuningMode.DEAD_ZONE;
        if (gamepad1.left_bumper && !lastLB) selectedMode = TuningMode.MAX_POWER;

        if (gamepad1.dpad_up && !lastUp) adjustValues(step);
        if (gamepad1.dpad_down && !lastDown) adjustValues(-step);

        if (gamepad1.dpad_right && !lastRight) step *= 10;
        if (gamepad1.dpad_left && !lastLeft) step /= 10;

        step = Range.clip(step, 1e-6, 10);

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastLB = gamepad1.left_bumper;
    }

    void adjustValues(double delta) {
        switch (selectedMode) {
            case P: Kp = Math.max(0, Kp + delta); break;
            case I: Ki = Math.max(0, Ki + delta); break;
            case D: Kd = Math.max(0, Kd + delta); break;
            case MAX_POWER: maxPower = Range.clip(maxPower + delta, 0, 1.0); break;
            case DEAD_ZONE: deadZone = Math.max(0, deadZone + delta); break;
        }
    }

    void telemetryDataDisplay(Double error, String status) {
        telemetry.addData("status", status);
        telemetry.addData("auto aim enabled", autoAimStatus);
        telemetry.addData("selected parameter", selectedMode);
        telemetry.addData("step size", step);
        telemetry.addData("kp", "%.6f", Kp);
        telemetry.addData("ki", "%.6f", Ki);
        telemetry.addData("kd", "%.6f", Kd);
        telemetry.addData("maxPower", "%.3f", maxPower);
        telemetry.addData("deadZone", "%.3f", deadZone);

        if (error != null) {
            telemetry.addData("tx", error);
        }

        if (limelight != null) {
            if (limelight.isConnected()) {
                telemetry.addLine("limelight connected");
            } else {
                telemetry.addLine("limelight disconnected");
            }
        }
        telemetry.update();
    }
}

