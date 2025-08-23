package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class mecanumDriveSubsystem extends SubsystemBase {

    public final Motor m_Fl, m_Fr, m_Rl, m_Rr;
    public double fwdPower;
    public double strPower;
    public double rotPower;

    public mecanumDriveSubsystem (Motor Fl, Motor Fr,
                                  Motor Rl, Motor Rr){
        m_Fl = Fl;
        m_Fr = Fr;
        m_Rl = Rl;
        m_Rr = Rr;

    }

    public void drive(double forward, double strafe, double rotation){
        fwdPower = forward;
        strPower = strafe;
        rotPower = rotation;

        double Fl = forward + strafe + rotation;
        double Fr = forward - strafe - rotation;
        double Rl = forward + strafe - rotation;
        double Rr = forward - strafe + rotation;

        //Normalize the powers
        double max = Math.max(1.0, Math.max(Math.abs(Fl),
                Math.max(Math.abs(Fr), Math.max(Math.abs(Rl), Math.abs(Rr)))));

        //set motor powers
        m_Fl.set(Fl / max);
        m_Fr.set(Fr / max);
        m_Rl.set(Rl / max);
        m_Rr.set(Rr / max);


    }

    public double getFwdPower(){
        return fwdPower;
    }

    public double getStrPower(){
        return strPower;
    }

    public double getRotPower(){
        return rotPower;
    }

    public Motor getM_Fl() {
        return m_Fl;
    }

    public Motor getM_Fr() {
        return m_Fr;
    }

    public Motor getM_Rl() {
        return m_Rl;
    }

    public Motor getM_Rr() {
        return m_Rr;
    }
}
