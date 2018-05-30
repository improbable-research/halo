import com.google.gson.Gson
import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import java.io.BufferedReader
import java.io.FileReader

class HelioStatCalibration : ArrayList<HelioStatCalibration.DataPoint> {
    class DataPoint(val ABC : Vector3D, val control: ServoSetting) {}

    val probabilisticHelioStat = HelioStat(ProbabilisticHelioStatParameters())


    constructor(array : List<DataPoint>) : super(array) {}
    constructor() : super() {}

    fun readFromFile(filename: String) {
        val gson = Gson()
        var buff = BufferedReader(FileReader(filename))

        val data = gson.fromJson(buff, CalibrationRawData::class.java)

        for(entry in data) {
            println(entry.key)
            println(entry.value.servoPositions)
            println("${entry.value.servoPositions["209"]?:0}, ${entry.value.servoPositions["210"]?:0}")
            println(entry.value.data.plane.ABCD.split(",").map(String::toDouble))
            val abcd = entry.value.data.plane.ABCD.split(",").map(String::toDouble)
            val plane = Vector3D(abcd[0], abcd[1], abcd[2]).normalize().scalarMultiply(abcd[3])
            println(plane)
            add(
                    DataPoint(
                            plane,
                            ServoSetting(entry.value.servoPositions["209"]?:0, entry.value.servoPositions["210"]?:0)
                    )
            )
        }

    }

    fun createBayesNet() : BayesNet {
        var i = 0
        for(dataPoint in this) {
            i += 1
            if(i%100 == 0) {
                val plane = probabilisticHelioStat.computeHeliostatPlane(
                        ConstantDoubleVertex(dataPoint.control.pitch.toDouble()),
                        ConstantDoubleVertex(dataPoint.control.rotation.toDouble())
                )
                plane.noisyObserve(dataPoint.ABC, Vector3D(0.01, 0.01, 0.01))
            }
        }
        return BayesNet(probabilisticHelioStat.params.cPitch.connectedGraph)
    }

    fun inferMaxAPosteriori() : HelioStatParameters {
        val model = createBayesNet()
        val optimiser = GradientOptimizer(model)
       optimiser.maxAPosteriori(10000)
        return probabilisticHelioStat.params.getValue()
    }


    fun calculateResiduals(params : HelioStatParameters) : Double {
        val forwardModel = HelioStat(ProbabilisticHelioStatParameters(params))
        var residual = 0.0
        var n = 0
        for(dataPoint in this) {
            val plane = forwardModel.computeHeliostatPlane(
                    ConstantDoubleVertex(dataPoint.control.pitch.toDouble()),
                    ConstantDoubleVertex(dataPoint.control.rotation.toDouble())
            )
            val modelledPlane = plane.getValue()
            println("modelled plane: $modelledPlane dataPoint: ${dataPoint.ABC}")
            residual += modelledPlane.subtract(dataPoint.ABC).norm
            n += 1
        }
        return residual/n
    }
}