package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

import java.util.function.Supplier;

public class teleOpMecanumDriveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final mecanumDriveSubsystem driveSub;
    private final Supplier<Double> xSupplier, ySupplier, rSupplier;



    public teleOpMecanumDriveCommand(mecanumDriveSubsystem driveSub,
                                     Supplier<Double> xSupplier, Supplier<Double> ySupplier,
                                     Supplier<Double> rSupplier){
        this.driveSub = driveSub;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        addRequirements(driveSub);
    }

    @Override
    public void execute(){

        driveSub.drive(ySupplier.get(), xSupplier.get(), rSupplier.get());
    }


    }

