package org.firstinspires.ftc.teamcode.controllers.arm

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings
import org.firstinspires.ftc.teamcode.motionProfiles.SCurveMotionProfile
import kotlin.math.*


open class Arm : Controller() {
    private val lowerAnglePID = PID(PIDSettings(kP = 0.03, kI = 0.0, kD = 0.0001))
    private val upperAnglePID = PID(PIDSettings(kP = 0.015, kI = 0.0, kD = 0.0001))

    private lateinit var lowerMotor: DcMotor
    private lateinit var upperMotor: DcMotor

    private lateinit var lowerAngle: AnalogInput
    private lateinit var upperAngle: AnalogInput

    private lateinit var intakeLeft: DcMotor
    private lateinit var intakeRight: DcMotor

    // Motion profile stuff

    private var upperMotionStartTime = 0.0
    private var lowerMotionStartTime = 0.0

    private var lowerMotionProfile = generateMotionProfile(0.0)
    private var upperMotionProfile = generateMotionProfile(0.0)

    private var upperMotionActive = false
    private var lowerMotionActive = false

    // End of motion profile stuff

    var upperSpeedLimit = 1.0
    var lowerSpeedLimit = 1.0

    var upperAngleTarget = 0.0
    set(value) {
        field = value
        upperMotionProfile = generateMotionProfile(field - currentAngles.upperAngle)
        upperMotionActive = true
        upperMotionStartTime = System.nanoTime().toDouble()
    }
    var lowerAngleTarget = 0.0
    set(value) {
        field = value
        upperMotionProfile = generateMotionProfile(field - currentAngles.upperAngle)
        lowerMotionActive = true
        lowerMotionStartTime = System.nanoTime().toDouble()
    }

    val currentAngles: ArmAngleValues
        get() {
            val angles = ArmAngleValues()

            angles.lowerAngle = 165.0 - 49.0 * lowerAngle.voltage - 37.1 * Math.pow(lowerAngle.voltage, 2.0) + 26.1 * Math.pow(lowerAngle.voltage, 3.0) - 4.58 * Math.pow(lowerAngle.voltage, 4.0)
            val x = upperAngle.voltage
            angles.upperAngle = -1.66243031965927 * x.pow(3) - 9.090297864880776 * x.pow(2) + 130.359681271249 * x - 137.040643577643

            return angles
        }

    fun generateMotionProfile(throwValue: Double): SCurveMotionProfile {
        return SCurveMotionProfile(100.0, 360.0, throwValue, 1.0)
    }

    override fun init(hardwareMap: HardwareMap) {
        lowerMotor = hardwareMap.get(DcMotor::class.java, "lowerMotor")
        upperMotor = hardwareMap.get(DcMotor::class.java, "upperMotor")

        intakeLeft = hardwareMap.get(DcMotor::class.java, "intakeLeft")
        intakeRight = hardwareMap.get(DcMotor::class.java, "intakeRight")

        lowerAngle = hardwareMap.get(AnalogInput::class.java, "lowerAngle")
        upperAngle = hardwareMap.get(AnalogInput::class.java, "upperAngle")


        lowerMotor.direction = DcMotorSimple.Direction.REVERSE
        upperMotor.direction = DcMotorSimple.Direction.REVERSE

        intakeLeft.direction = DcMotorSimple.Direction.REVERSE
        intakeRight.direction = DcMotorSimple.Direction.REVERSE

        lowerAnglePID.target = currentAngles.lowerAngle
        upperAnglePID.target = currentAngles.upperAngle
    }

    override fun update(timeStep: Double) {

        if (currentAngles.lowerAngle < 80 && upperAngleTarget < -45) {
            upperAngleTarget = -45.0
        }


        if (lowerAngleTarget < 35) lowerAngleTarget = 35.0

        if(upperMotionActive){
            val upperTimeDelta = System.nanoTime() - upperMotionStartTime
            lowerAnglePID.target = upperMotionProfile.getPosition(upperTimeDelta)
            if(upperMotionProfile.getTimeNeeded() < upperTimeDelta){
                upperMotionActive = false
            }
        }

        if(lowerMotionActive){
            val lowerTimeDelta = System.nanoTime() - lowerMotionStartTime
            lowerAnglePID.target = lowerMotionProfile.getPosition(lowerTimeDelta)
            if(lowerMotionProfile.getTimeNeeded() < lowerTimeDelta){
                lowerMotionActive = false
            }
        }

        val lowerOutput = lowerAnglePID.update(currentAngles.lowerAngle, timeStep)
        val upperOutput = upperAnglePID.update(currentAngles.upperAngle, timeStep)

        //telemetry.addData("currentAngles", currentAngles)

        telemetry.addData("lowerAngle", currentAngles.lowerAngle)
        telemetry.addData("upperAngle", currentAngles.upperAngle)

        telemetry.addData("lowerError", lowerAnglePID.error)
        telemetry.addData("upperError", upperAnglePID.error)

        lowerMotor.power = if (lowerOutput.absoluteValue > lowerSpeedLimit) lowerOutput.sign * lowerSpeedLimit else lowerOutput
        upperMotor.power = if (upperOutput.absoluteValue > upperSpeedLimit) upperOutput.sign * upperSpeedLimit else upperOutput
    }
}

