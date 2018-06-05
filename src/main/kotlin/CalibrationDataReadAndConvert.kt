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

    fun readFromFileFormat2(filename: String) {
        val gson = Gson()
        var buff = BufferedReader(FileReader(filename))
        val data = gson.fromJson(buff, CalibrationRawData2::class.java)

        for(entry in data.captures) {
            val abcd = entry.ABCD
            var length = abcd[3]
            var cartesianPlane = Vector3D(abcd[0], abcd[1], abcd[2]).normalize()
            if(abcd[1] < 0.0) {
                cartesianPlane = cartesianPlane.scalarMultiply(-1.0)
               length *= -1.0
            }
            val sphericalPlane = Geometry.cartesianToSpherical(cartesianPlane)
            add(HelioStatCalibration.DataPoint(length,
                    sphericalPlane.y,
                    sphericalPlane.z,
                    ServoSetting(entry.A1, entry.A2)))
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
            var pitch = rand.nextDouble() * 0.5 * PI
            var rotation = (rand.nextDouble()-0.5) * 2.0 * PI
            var normal = Geometry.sphericalToCartesian(Vector3D(1.0, pitch, rotation))
            val control = HelioStatNavigator(params).normalToServoSignal(normal)

            normal = forwardModel.computeHeliostatNormal(control).getValue()
            val sphericalNormal = Geometry.cartesianToSpherical(normal)
            pitch = sphericalNormal.y
            rotation = sphericalNormal.z

            val planeDist = forwardModel.computePlaneDistanceFromOrigin(pitch, rotation)

            // TODO add some gaussian noise and see how it performs as the noise increases
            // TODO See how it performs with fewer points
            // TODO Look at the residual and ensure it's just noise

//            println("Err is ${Geometry.cartesianToSpherical(normal).subtract(Vector3D(1.0, pitch, rotation))}")

            this.add(HelioStatCalibration.DataPoint(planeDist.value, pitch, rotation, control))
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
    c.readFromFileFormat2("heliostatData.json")
    for(entry in c) {
        println(entry)
    }
}
