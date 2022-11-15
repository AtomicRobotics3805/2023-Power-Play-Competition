package org.firstinspires.ftc.teamcode.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.atomicrobotics.cflib.Command
import com.atomicrobotics.cflib.Constants
import com.atomicrobotics.cflib.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

@Config
@Suppress("Unused", "MemberVisibilityCanBePrivate")
object OpenCVWebcam : Subsystem {

    @JvmField
    var NAME = "Webcam 1"
    @JvmField
    var cameraMonitorViewId: Int = Constants.opMode.hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId",
            "id",
            Constants.opMode.hardwareMap.appContext.getPackageName()
    )

    private lateinit var camera: OpenCvWebcam
    private lateinit var pipeline: PowerPlayPipeline

    var detectedColor = PowerPlayPipeline.SleeveColor.UNDETECTED

    val color: PowerPlayPipeline.SleeveColor
        get() = pipeline.color

    val detect: DetectCommand
        get() = DetectCommand()

    class DetectCommand : Command(){
        override val _isDone: Boolean
            get() = detectedColor != PowerPlayPipeline.SleeveColor.UNDETECTED

        override fun start(){
            detectedColor = PowerPlayPipeline.SleeveColor.UNDETECTED
        }

        override fun execute(){
            camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
                override fun onOpened() {
                    camera.startStreaming(1920, 1080, OpenCvCameraRotation.SIDEWAYS_LEFT)
                }

                override fun onError(errorCode: Int) {
                }
            })
        }
    }

    override fun initialize(){
        pipeline = PowerPlayPipeline()
        camera = OpenCvCameraFactory.getInstance().createWebcam(Constants.opMode.hardwareMap.get<WebcamName>(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
        camera.setPipeline(pipeline)
    }

}

class PowerPlayPipeline : OpenCvPipeline() {

    enum class SleeveColor {
        YELLOW,
        CYAN,
        MAGENTA,
        UNDETECTED
    }

    private val YELLOW = Scalar(255.0, 255.0, 0.0)
    private val CYAN = Scalar(0.0, 255.0, 255.0)
    private val MAGENTA = Scalar(255.0, 0.0, 255.0)

    public var color : SleeveColor = SleeveColor.UNDETECTED

    var pointA = Point(0.0, 1400.0)
    var pointB = Point(300.0, 1100.0)
    var ROI = Rect(pointA,pointB)

    override fun processFrame(input : Mat): Mat? {
        val mat = input.submat(ROI)
        val sumColors = Core.sumElems(mat)
        val minColor = Math.min(sumColors.`val`[0], Math.min(sumColors.`val`[1], sumColors.`val`[2]))
        if (sumColors.`val`[0] == minColor) {
            color = SleeveColor.CYAN
            Imgproc.rectangle(
                    input,
                    pointA,
                    pointB,
                    CYAN,
                    3
            )
        } else if (sumColors.`val`[1] == minColor) {
            color = SleeveColor.MAGENTA
            Imgproc.rectangle(
                    input,
                    pointA,
                    pointB,
                    MAGENTA,
                    3
            )
        } else {
            color = SleeveColor.YELLOW
            Imgproc.rectangle(
                    input,
                    pointA,
                    pointB,
                    YELLOW,
                    3
            )
        }

        return mat
    }
}