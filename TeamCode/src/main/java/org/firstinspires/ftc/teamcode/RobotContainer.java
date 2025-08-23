package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

@TeleOp (name = "TeleOpMode")
public class RobotContainer extends CommandOpMode {
    private mecanumDriveSubsystem driveSub;
    private GamepadEx driverJoystick;

    @Override
    public void initialize() {
        driveSub = new mecanumDriveSubsystem(
                new Motor(hardwareMap, "FrontLeft"),
                new Motor(hardwareMap, "FrontRight"),
                new Motor(hardwareMap, "RearLeft"),
                new Motor(hardwareMap, "RearRight"),
                hardwareMap
        );

        driverJoystick = new GamepadEx(gamepad1);

        runCommands();
        setDefaultCommands();
    }

    public void setDefaultCommands(){

        //sets default that joystick's will always be able to drive unless autonomous
        driveSub.setDefaultCommand(
                new teleOpMecanumDriveCommand(
                        driveSub,
                        () -> driverJoystick.getLeftX(),
                        () -> -driverJoystick.getLeftY(),
                        () -> driverJoystick.getRightX()
                )
        );
    }

    private void runCommands(){

    }


}
