import com.google.gson.Gson
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.io.BufferedReader
import java.io.FileReader
import java.util.*

class CalibrationDataReadAndConvert : ArrayList<HelioStatCalibration.DataPoint> () {

    val rand = Random()

    fun readFromFile(filename: String) {
        val gson = Gson()
        var buff = BufferedReader(FileReader(filename))

        val data = gson.fromJson(buff, CalibrationRawData::class.java)

        for(entry in data) {
//            println(entry.key)
//            println(entry.value.servoPositions)
//            println("${entry.value.servoPositions["209"]?:0}, ${entry.value.servoPositions["210"]?:0}")
//            println(entry.value.data.plane.ABCD.split(",").map(String::toDouble))
            val abcd = entry.value.data.plane.ABCD.split(",").map(String::toDouble)
            val length = abcd[3]
            val cartesianPlane = Vector3D(abcd[0], abcd[1], abcd[2]).normalize()
//            println("cartesian is $cartesianPlane")
            val sphericalPlane = Geometry.cartesianToSpherical(cartesianPlane)
//            println(sphericalPlane)
            add(HelioStatCalibration.DataPoint(length,
                                               sphericalPlane.y,
                                               sphericalPlane.z,
                                               ServoSetting(entry.value.servoPositions["209"] ?: 0,
                                                              entry.value.servoPositions["210"] ?: 0)))
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

}

fun main(args : Array<String>) {
    var c = CalibrationDataReadAndConvert()
    c.readFromFile("calibrationData.json")
    println("" + c[2].length + " " + c[2].pitch + " " + c[2].rotation + " " + c[2].control.pitch + " " + c[2].control.rotation)
}
