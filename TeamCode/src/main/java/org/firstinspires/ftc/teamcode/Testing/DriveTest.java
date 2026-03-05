package org.firstinspires.ftc.teamcode.Testing;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;

public class DriveTest extends CommandOpMode
{
    Follower follower;

    @Override
    public void initialize()
    {
        follower = PPConstants.createFollower(hardwareMap);
    }

    boolean started = false;

    @Override
    public void run()
    {
        if (!started)
        {
            follower.startTeleopDrive();
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                true
        );
    }

}
