import com.google.gson.Gson
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.io.BufferedReader
import java.io.FileReader
import java.util.*
import kotlin.math.PI

class CalibrationData : ArrayList<HelioStatCalibration.DataPoint>() {

    val rand = Random()

    fun readFromFile(filename: String) {
        val gson = Gson()
        var buff = BufferedReader(FileReader(filename))
        val data = gson.fromJson(buff, CalibrationRawData::class.java)

        for (entry in data.entries) {
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

        for (entry in data.captures) {
            val abcd = entry.ABCD
            var length = abcd[3]
            var cartesianPlane = Vector3D(abcd[0], abcd[1], abcd[2]).normalize()
            if (abcd[1] < 0.0) {
                cartesianPlane = cartesianPlane.scalarMultiply(-1.0)
                length *= -1.0
            }
            var sphericalPlane = Geometry.cartesianToSpherical(cartesianPlane)
            if (entry.A2 > 2100) {
                sphericalPlane = Geometry.erectToFlacid(sphericalPlane)
            }
            if (sphericalPlane.z < -Math.PI / 2) sphericalPlane = Vector3D(sphericalPlane.x, sphericalPlane.y, sphericalPlane.z + 2.0 * Math.PI)
            add(HelioStatCalibration.DataPoint(length,
                    sphericalPlane.y,
                    sphericalPlane.z,
                    ServoSetting(entry.A1, entry.A2)))
        }
    }

    fun randomSubSample(nSamples: Int): CalibrationData {
        var n = nSamples
        var i = 0
        val subset = ArrayList<HelioStatCalibration.DataPoint>()
        while (n-- > 0 && size > 0) {
            i = rand.nextInt(this.size)
            subset.add(this.removeAt(i))
        }

        val subSample = CalibrationData()
        subSample.addAll(subset)
        return subSample
    }

    fun createSyntheticTrainingSet(nSamples: Int, params: HelioStatParameters) {
        val forwardModel = HelioStat(ProbabilisticHelioStatParameters(params))
        this.clear()
        for (i in 1..nSamples) {
            var pitch = (rand.nextDouble() - 0.5) * PI
            var rotation = rand.nextDouble() * PI
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
        println("${value.servoPositions["209"] ?: 0}, ${value.servoPositions["210"] ?: 0}")
        println(value.data.plane.ABCD.split(",").map(String::toDouble))
        println("cartesian is $cartesianPlane")
        println(sphericalPlane)
    }
}

fun main(args: Array<String>) {
    var c = CalibrationData()
    c.readFromFileFormat2("heliostatData.json")
    c.createSyntheticTrainingSet(40, HelioStatParameters(
            Vector3D(1.0, 1.0, 1.0),
            HelioStatParameters.ServoParameters(0.001, 0.1, Math.PI / 2.0, -Math.PI / 2.0),
            HelioStatParameters.ServoParameters(0.002, 0.2, Math.PI, 0.0)
    ))
    for (entry in c) {
        println(entry)
    }
}
