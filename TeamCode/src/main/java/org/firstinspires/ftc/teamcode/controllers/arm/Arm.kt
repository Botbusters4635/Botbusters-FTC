package org.firstinspires.ftc.teamcode.controllers.arm

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.ValueCacher
import org.firstinspires.ftc.teamcode.controllers.motors.EctoDcMotor
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import org.firstinspires.ftc.teamcode.motionProfiles.SCurveMotionProfile
import kotlin.math.*


open class Arm : Controller() {
    private val lowerAnglePID = PID(PIDSettings(kP = 0.05, kI = 0.0, kD = 0.000))
    private val upperAnglePID = PID(PIDSettings(kP = 0.01857, kI = 0.008, kD = 0.000012, iClearZone = 5.0))

    private val lowerMotor = EctoDcMotor("lowerMotor")
    private val upperMotor = EctoDcMotor("upperMotor")

    private lateinit var lowerAngle: AnalogInput
    private lateinit var upperAngle: AnalogInput

    private var lastLowerTarget = 0.0
    private var lastUpperTarget = 0.0

    // Motion profile stuff

    private val upperMotorSpeedLimit = 0.85
    private val lowerMotorSpeedLimit = 0.85

    private var upperMotionStartTime = 0.0
    private var lowerMotionStartTime = 0.0

    private var lowerMotionProfile = generateMotionProfile(0.0, 0.0)
    private var upperMotionProfile = generateMotionProfile(0.0, 0.0)

    private var upperMotionActive = false
    private var lowerMotionActive = false

    // End of motion profile stuff

    var upperSpeedLimit = 1.0
    var lowerSpeedLimit = 0.5

    var upperAngleTarget = -40.0
    var lowerAngleTarget = 90.0

    private var lastTimeAnglesRead = SystemClock.elapsedRealtime() / 1000.0
    private var lastAngleReading = ArmAngleValues()

    private val currentAngleCacher = ValueCacher(ArmAngleValues())
    val currentAngles: ArmAngleValues
        get() = currentAngleCacher.cachedGet{
            val angles = ArmAngleValues()
//
            angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0) - 10.0
            val x = upperAngle.voltage
            angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643 + 23

            angles
        }()


    fun generateMotionProfile(throwValue: Double, startPos : Double): SCurveMotionProfile {
        return SCurveMotionProfile(400.0, 200.0, throwValue, 1.0, startPos)
    }

    override fun init(hardwareMap: HardwareMap) {
        lowerAngle = hardwareMap.get(AnalogInput::class.java,  "lowerAngle")
        upperAngle = hardwareMap.get(AnalogInput::class.java, "upperAngle")
        controllers.add(lowerMotor)
        controllers.add(upperMotor)

//        lowerMotor.direction = DcMotorSimple.Direction.REVERSE
//        upperMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun update(timeStep: Double) {
        var currentLowerTarget = lowerAngleTarget
        var currentUpperTarget = upperAngleTarget

        if (currentAngles.lowerAngle < 65 && currentUpperTarget < -45) {
            currentUpperTarget = -45.0
        }

        if (currentLowerTarget < 35) currentLowerTarget = 35.0

        if(currentUpperTarget != lastUpperTarget){
            upperMotionProfile = generateMotionProfile(currentUpperTarget - currentAngles.upperAngle, currentAngles.upperAngle)
            upperMotionActive = true
            upperMotionStartTime = SystemClock.elapsedRealtime() / 1000.0
        }

        if(currentLowerTarget != lastLowerTarget){
            lowerMotionProfile = generateMotionProfile(currentLowerTarget - currentAngles.lowerAngle, currentAngles.lowerAngle)
            lowerMotionActive = true
            lowerMotionStartTime = SystemClock.elapsedRealtime() / 1000.0
        }



        lowerAnglePID.target = lowerMotionProfile.getPosition(SystemClock.elapsedRealtime() / 1000.0 - lowerMotionStartTime)

        upperAnglePID.target = upperMotionProfile.getPosition(SystemClock.elapsedRealtime() / 1000.0 - upperMotionStartTime)

        lastLowerTarget = currentLowerTarget

        lastUpperTarget = currentUpperTarget
//
//        lowerAnglePID.target = 90.0
//
//        upperAnglePID.target = 0.0

        val lowerOutput = lowerAnglePID.update(currentAngles.lowerAngle, timeStep)
        val upperOutput = upperAnglePID.update(currentAngles.upperAngle, timeStep)
//
        telemetry.addData("lowerAngle", currentAngles.lowerAngle)
        telemetry.addData("upperAngle", currentAngles.upperAngle)
//
        telemetry.addData("lowerTarget", lowerAnglePID.target)
        telemetry.addData("upperTarget", upperAnglePID.target)
        telemetry.addData("upperOutRaw", upperOutput)
        telemetry.addData("upperDeltaError", upperAnglePID.deltaError)


        val lowerFinalOutput = if (lowerOutput.absoluteValue > lowerSpeedLimit) lowerOutput.sign * lowerSpeedLimit else lowerOutput

        val upperFinalOutput = if (upperOutput.absoluteValue > upperSpeedLimit) upperOutput.sign * upperSpeedLimit else upperOutput


        telemetry.addData("upperOut", -upperFinalOutput)
        telemetry.addData("lowerOut", -lowerFinalOutput)
        telemetry.addData("lowerErr", lowerAnglePID.error)
        telemetry.addData("upperErr", upperAnglePID.error)


        writeMotors(-lowerFinalOutput, -upperFinalOutput)

    }


    fun writeMotors(lowerPower: Double, upperPower: Double) {
        lowerMotor.power = lowerPower * lowerMotorSpeedLimit
        upperMotor.power = upperPower * upperMotorSpeedLimit

    }
}

