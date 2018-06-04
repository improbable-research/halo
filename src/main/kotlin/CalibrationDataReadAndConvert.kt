import com.google.gson.Gson
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.io.BufferedReader
import java.io.FileReader
import java.lang.Math.abs
import java.util.*
import kotlin.math.PI
import kotlin.math.roundToInt

class CalibrationDataReadAndConvert : ArrayList<HelioStatCalibration.DataPoint> () {

    val rand = Random()

    fun readFromFile(filename: String) {
        val gson = Gson()
        var buff = BufferedReader(FileReader(filename))
        val data = gson.fromJson(buff, CalibrationRawData::class.java)

        for(entry in data.entries) {
            val abcd = entry.value.data.plane.ABCD.split(",").map(String::toDouble)
            val length = abcd[3]
            val cartesianPlane = Vector3D(abcd[0], abcd[1], abcd[2]).normalize()
            val sphericalPlane = Geometry.cartesianToSpherical(cartesianPlane)
            add(HelioStatCalibration.DataPoint(length,
                                               sphericalPlane.y,
                                               sphericalPlane.z,
                                               ServoSetting(entry.value.servoPositions["209"] ?: 0,
                                                              entry.value.servoPositions["210"] ?: 0)))
            //debug_printEntries(entry.key, entry.value, cartesianPlane, sphericalPlane)
        }
    }

    fun randomSubSample(nSamples : Int) {
        var n = nSamples
        var i = 0
        val subset = ArrayList<HelioStatCalibration.DataPoint>()
        while(n-- > 0 && size > 0) {
            i = rand.nextInt(this.size)
            subset.add(this.removeAt(i))
        }
        this.clear()
        this.addAll(subset)
    }

    fun createSyntheticTrainingSet(nSamples : Int, params : HelioStatParameters) {
        val forwardModel = HelioStat(ProbabilisticHelioStatParameters(params))
        this.clear()
        for (i in 1..nSamples) {
            val control = ServoSetting(
                    ((rand.nextDouble() * 2.0 * PI - params.rotationParameters.c) / params.rotationParameters.m).roundToInt(),
                    (((rand.nextDouble()-0.5) * 2.0 * PI - params.pitchParameters.c) / params.pitchParameters.m).roundToInt()
            )

            // TODO add some gaussian noise and see how it performs as the noise increases
            // TODO See how it performs with fewer points
            // TODO Look at the residual and ensure it's just noise
            val plane = forwardModel.computeHeliostatPlaneSpherical(
                    ConstantDoubleVertex(control.pitch.toDouble()),
                    ConstantDoubleVertex(control.rotation.toDouble())
            )
            plane.x.lazyEval()
            plane.y.lazyEval()
            plane.z.lazyEval()

            var modelledPlaneSpherical = plane.getValue()
//            if (control.pitch < 0) {
//                modelledPlaneSpherical = Geometry.erectToFlacid(modelledPlaneSpherical)
//            }
//            println("modelled plane: $modelledPlaneSpherical")
            this.add(HelioStatCalibration.DataPoint(modelledPlaneSpherical.x, modelledPlaneSpherical.y, modelledPlaneSpherical.z, control))
        }
    }

    fun debug_printEntries(key: String, value: CalibrationRawData.DataPoint, cartesianPlane: Vector3D, sphericalPlane: Vector3D) {
            println(key)
            println(value.servoPositions)
            println("${value.servoPositions["209"]?:0}, ${value.servoPositions["210"]?:0}")
            println(value.data.plane.ABCD.split(",").map(String::toDouble))
            println("cartesian is $cartesianPlane")
            println(sphericalPlane)
    }
}

fun main(args : Array<String>) {
    var c = CalibrationDataReadAndConvert()
    c.readFromFile("calibrationData.json")
    for(entry in c) {
        if(abs(entry.length) < 1.0) println("" + entry.control.pitch + " " + entry.control.rotation + " " +
                entry.length + " " + entry.pitch + " " + entry.rotation)
    }
}
