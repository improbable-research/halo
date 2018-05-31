import com.google.gson.Gson
import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.optim.SimpleValueChecker
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer
import java.io.BufferedReader
import java.io.FileReader
import java.util.*
import kotlin.math.PI

class HelioStatCalibration : ArrayList<HelioStatCalibration.DataPoint> {
    class DataPoint(val ABC : Vector3D, val control: ServoSetting) {}

    val probabilisticHelioStat = HelioStat(ProbabilisticHelioStatParameters())
    val rand = Random()


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
        for(dataPoint in this) {
            val plane = probabilisticHelioStat.computeHeliostatPlane(
                    ConstantDoubleVertex(dataPoint.control.pitch.toDouble()),
                    ConstantDoubleVertex(dataPoint.control.rotation.toDouble())
            )
            plane.noisyObserve(dataPoint.ABC, Vector3D(0.01, 0.01, 0.01))
        }
        return BayesNet(probabilisticHelioStat.params.cPitch.connectedGraph)
    }

    fun inferPivotPoint() : Vector3D {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())
        helioStat.params.pivotPoint.x.value = 1.0
        helioStat.params.pivotPoint.y.value = 1.0
        helioStat.params.pivotPoint.z.value = 1.0

        for(dataPoint in this) {
            val norm = helioStat.computePlaneNorm(ProbabilisticVector3D(dataPoint.ABC.scalarMultiply(1.0/dataPoint.ABC.norm)))
            println("norm: ${norm.value} ${dataPoint.ABC.norm} ${norm.value - dataPoint.ABC.norm}")
            GaussianVertex(norm, 0.001).observe(dataPoint.ABC.norm)
        }
        val model = BayesNet(helioStat.params.pivotPoint.x.connectedGraph)
        val optimiser = GradientOptimizer(model)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-16, 1e-16)
                )
        )
        return(helioStat.params.pivotPoint.getValue())
    }

    fun inferMaxAPosteriori() : HelioStatParameters {
        val model = createBayesNet()
        val optimiser = GradientOptimizer(model)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-16, 1e-16)
                )
        )
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

    fun randomSubSample(nSamples : Int) {
        var n = nSamples
        var i = 0;
        val subset = ArrayList<DataPoint>()
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
        for(i in 1..nSamples) {
            val control = ServoSetting(
                    ((rand.nextDouble()*2.0*PI - params.rotationParameters.c)/params.rotationParameters.m).toInt(),
                    ((rand.nextDouble()*2.0*PI - params.pitchParameters.c)/params.pitchParameters.m).toInt()
            )

            val plane = forwardModel.computeHeliostatPlane(
                    ConstantDoubleVertex(control.pitch.toDouble()),
                    ConstantDoubleVertex(control.rotation.toDouble())
            )
            plane.x.lazyEval()
            plane.y.lazyEval()
            plane.z.lazyEval()

            val modelledPlane = plane.getValue()
            println("modelled plane: $modelledPlane")
            this.add(DataPoint(plane.getValue(), control))
        }

    }

}