package org.firstinspires.ftc.teamcode.controllers

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.Coordinate
import java.util.ArrayList

class VisionController : Controller() {
    lateinit var vuforia: VuforiaLocalizer
    lateinit var skystone: VuforiaTrackables
    lateinit var allTrackables: ArrayList<VuforiaTrackable>
    var lastLocation = Coordinate()
    var isVisible = false

    override fun init(hardwareMap: HardwareMap) {

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        parameters.vuforiaLicenseKey = "ASErNM3/////AAABmQ30vCYZm01JlyuwHXrdZGiEOn5z2wZ3z0Q9EHX2stCeX9ZyqkHxSWkw90Cnv6O/xSAi42GWir1yJiBExY43ZFuliHnTkfszjlmaSQCdpGjz11Os90Xtjf33x9jzuMLpmVnkTOuTkWbJmv/tnDfsoIBxJWNl0NTw3hd9e25sgmhRSPsjZ/cenBKpVkHinJxvqS1j8K3bjh9UiE6lcA7ND5SG81QQnI4FsOLaTzFMFHmU+t0qL7VJ5twoEkvpWqm3tbrC8CxD7ITrtQ2beRxwX5ENBLGfx1cfzurHKB04SADG1E1T5CESCL7Z7NbgGQM/RywNiDJTZCAb5A0jFbGn91tzH89bN+iQhlV+00q6rF4v"
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
        vuforia = ClassFactory.getInstance().createVuforia(parameters)
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true)


        skystone = this.vuforia.loadTrackablesFromAsset("Skystone")
        val targetElement = skystone[0]
        targetElement.name = "targetElement"  // Stones

//        val blueTarget = stonesAndChips[1]
//        blueTarget.name = "BlueTarget"  // Chips

        allTrackables = ArrayList()
        allTrackables.addAll(skystone)

        val mmPerInch = 25.4f
        val mmBotWidth = 18 * mmPerInch            // ... or whatever is right for your robot
        val mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch   // the FTC field is ~11'10" center-to-center of the glass panels

        val redTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(0f, 0f, 0f)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90f, 90f, 0f))
        targetElement.location = redTargetLocationOnField

        /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
//        val blueTargetLocationOnField = OpenGLMatrix
//                /* Then we translate the target off to the Blue Audience wall.
//                Our translation here is a positive translation in Y.*/
//                .translation(0f, mmFTCFieldWidth / 2, 0f)
//                .multiplied(Orientation.getRotationMatrix(
//                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90f, 0f, 0f))
//        blueTarget.location = blueTargetLocationOnField

        val phoneLocationOnRobot = OpenGLMatrix
                .translation(0f, 0f, 0f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90f, 0f, 0f))
        (targetElement.listener as VuforiaTrackableDefaultListener).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection)
//        (blueTarget.listener as VuforiaTrackableDefaultListener).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection)


    }

    override fun start() {
        skystone.activate()
    }

    override fun update(timeStep: Double) {
        var currentVisibility = false
        for (trackable in allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            val robotLocationTransform = (trackable.listener as VuforiaTrackableDefaultListener).updatedVuforiaCameraFromTarget
            val thingIsVisible = (trackable.listener as VuforiaTrackableDefaultListener).isVisible
            currentVisibility = if (currentVisibility) true else thingIsVisible
            if (robotLocationTransform != null) {
                lastLocation = Coordinate(-robotLocationTransform[2, 3].toDouble(), robotLocationTransform[0, 3].toDouble())
            }
        }
        isVisible = currentVisibility
        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
    }
}