package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class TelemetryManager {

    private final mecanumDriveSubsystem driveSub;
    private final teleOpMecanumDriveCommand teleOpDriveCmd;
    private final Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();

    public TelemetryManager(mecanumDriveSubsystem driveSub,
                            teleOpMecanumDriveCommand teleOpDriveCmd, Telemetry telemetry){
        this.driveSub = driveSub;
        this.teleOpDriveCmd = teleOpDriveCmd;
        this.telemetry = telemetry;
    }

    public void runTelemetry(){
        telemetry.addData("Initilized Time", "%.2f Secounds", timer.seconds());

        telemetry.addData("Heading (rad)", driveSub.getHeading());

        telemetry.addData("Power's", "Fwd: %.2f | Str: %.2f | Rot: %.2f",
                driveSub.getFwdPower(),
                driveSub.getStrPower(),
                driveSub.getRotPower());

        telemetry.update();
    }
}
