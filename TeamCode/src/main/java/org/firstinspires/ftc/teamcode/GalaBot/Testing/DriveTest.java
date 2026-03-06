package org.firstinspires.ftc.teamcode.GalaBot.Testing;

import static org.firstinspires.ftc.teamcode.GalaBot.ForwardPID.fD;
import static org.firstinspires.ftc.teamcode.GalaBot.ForwardPID.fI;
import static org.firstinspires.ftc.teamcode.GalaBot.ForwardPID.fP;
import static org.firstinspires.ftc.teamcode.GalaBot.YawPID.yD;
import static org.firstinspires.ftc.teamcode.GalaBot.YawPID.yI;
import static org.firstinspires.ftc.teamcode.GalaBot.YawPID.yP;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.GalaBot.Enums.LastDir;

import java.util.ArrayList;

@TeleOp (name = "Drive Test", group = "Test")
public class DriveTest extends CommandOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Limelight3A ll;
    private PIDController forwardPID;
    private PIDController yawPID;

    @Override
    public void initialize() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftD");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightD");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftD");
        backRight = hardwareMap.get(DcMotor.class, "backRightD");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(8);
        ll.start();

        forwardPID = new PIDController(fP, fI, fD);
        yawPID = new PIDController(yP, yI, yD);
    }

    private final double FOLLOWING_DISTANCE = 40.0;
    private final double CENTER_OFFSET = 0.0;

    private boolean started = false;

    private double distance = 40.0;
    private double Tx = 0.0;

    private ArrayList<Double> distLog = new ArrayList<>();
    private ArrayList<Double> xLog = new ArrayList<>();
    private ElapsedTime logTimer = new ElapsedTime();

    private ElapsedTime loopTimer = new ElapsedTime();

    private boolean firstCycle = true;

    private LastDir lastDir = LastDir.NONE;

    @Override
    public void run() {
        if (!started)
        {
            logTimer.reset();
            loopTimer.reset();
            started = true;
        }

        forwardPID.setPID(fP, fI, fD);
        yawPID.setPID(yP, yI, yD);

        LLResult result = ll.getLatestResult();
        if (firstCycle)
        {
            if (result.isValid())
            {
                distLog.add(getDistance(result.getTa()));
                xLog.add(result.getTx());
            } else
            {
                distLog.add(FOLLOWING_DISTANCE);
                xLog.add(CENTER_OFFSET);
            }
        }
        else
        {
            if (result.isValid())
            {
                distLog.add(getDistance(result.getTa()));
                xLog.add(result.getTx());
                distLog.remove(0);
                xLog.remove(0);
            } else
            {
                distLog.add(FOLLOWING_DISTANCE);
                xLog.add(CENTER_OFFSET);
                distLog.remove(0);
                xLog.remove(0);
            }
        }


        if (logTimer.milliseconds() >= 10)
        {
            logTimer.reset();
            firstCycle = false;
        }

        double distSum = 0.0;
        double xSum = 0.0;
        for (int i = 0; i < distLog.size(); i++)
        {
            distSum += distLog.get(i);
            xSum += xLog.get(i);
        }
        distance = distSum / distLog.size();
        Tx = xSum / xLog.size();

        double pow = forwardPID.calculate(distance, FOLLOWING_DISTANCE);
        double pow1 = yawPID.calculate(Tx, CENTER_OFFSET);

        if (Math.abs(Tx - CENTER_OFFSET) < 5 && !result.isValid())
        {
            double power;
            if (lastDir == LastDir.LEFT) power = -0.2;
            else if (lastDir == LastDir.RIGHT) power = 0.2;
            else power = 0;
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }
        else
        {
            frontLeft.setPower(pow + pow1);
            frontRight.setPower(pow + (-pow1));
            backLeft.setPower(pow + pow1);
            backRight.setPower(pow + (-pow1));

            if (pow1 > 0.1) lastDir = LastDir.RIGHT;
            else if (pow1 < 0.1) lastDir = LastDir.LEFT;
            else lastDir = LastDir.NONE;
        }



        telemetry.addData("TX", Tx);
        telemetry.addData("TA", result.getTa());
        telemetry.addData("DIST", distance);
        telemetry.addData("LOOP TIME", loopTimer.milliseconds());
        telemetry.update();

        loopTimer.reset();
    }

    public double getDistance(double Ta) {
        return 71.84527 * Math.pow(Ta, -0.5063194);
    }
}


