package org.firstinspires.ftc.teamcode.motionProfiles

import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.pow
import kotlin.math.sign

class SCurveMotionProfile (var Vm: Double , var Am : Double, var Throw : Double, var gamma : Double){
    val Sgn = Throw.sign * 1.0
    val Yf = abs(Throw)
    val Ys = Yf/2.0

    val Yaux = (1.0/2.0) * (1+gamma) * (Vm.pow(2) / Am)
    val Ya = if(Ys <= Yaux) Ys else Yaux;
    val Vw = if(Ys <= Yaux) sqrt((Ys * Am)/((1.0/2.0) * (1+gamma))) else Vm
    val To = Vw / Am
    val Ta = To * (1 + gamma)
    val tau = gamma * To
    val Tm = Ta / 2.0
    val Tk = 2.0 * (Ys - Ya) / Vm
    val Ts = Ta + Tk / 2.0
    val Tt = 2.0 * Ts

    fun getPosition(time : Double) : Double{
        return Sgn * position_base(time)
    }

    fun getVelocity(time : Double ) : Double{
        return Sgn * velocity_base(time)
    }

    fun getTimeNeeded() : Double{
        return Tt;
    }

    private fun position_base(time : Double ) : Double{
        if(time > 0 && time < tau){
            return ((1.0/2.0) * (Vw / To)) * (time.pow(3) / (3.0 * tau))
        }

        if(time > tau && time <= To){
            return ((1.0/2.0) * (Vw / To)) * ((time - (tau / 2.0)).pow(2) + tau.pow(2) / 12.0)
        }

        if(time > To && time <= Ts){
            return Ys + Vw * (time - Ts) + position_base(Ta - time)
        }

        if(time > Ts){
            return Yf - position_base(Tt - time)
        }
        return 0.0
    }

    private fun velocity_base(time : Double ) : Double {
        if(time > 0 && time < tau){
            return (Am / 2.0) * (time.pow(2) / tau)
        }

        if(time > tau && time <= To){
            return (Am / 2.0) * (2.0 * time - tau)
        }

        if(time > Tm && time <= Ts){
            return Vw - velocity_base(Ta - time)
        }

        if(Ts < time){
            return velocity_base(Tt - time)
        }
        return 0.0;
    }
}