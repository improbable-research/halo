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
import java.lang.Math.*
import java.util.*
import kotlin.math.PI

class HelioStatCalibration : ArrayList<HelioStatCalibration.DataPoint> {
    class DataPoint( val length : Double, val pitch : Double, val rotation : Double, val control: ServoSetting) {}

    constructor(data : Array<DataPoint>) {
        this.addAll(data)
    }

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
            // TODO fix this for spherical
//            add(
//                    DataPoint(
//                            plane,
//                            ServoSetting(entry.value.servoPositions["209"]?:0, entry.value.servoPositions["210"]?:0)
//                    )
//            )
        }

    }

    fun inferAllParams() : HelioStatParameters {
        val params = inferServoParams()
        params.pivotPoint = inferPivotPoint()
        return params
    }

    fun inferPivotPoint() : Vector3D {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())

        for(dataPoint in this) {
            val modelledLength = helioStat.computePlaneDistanceFromOrigin(dataPoint.pitch,dataPoint.rotation)
//            println("len: ${modelledLength.value} ${dataPoint.length} ${modelledLength.value - dataPoint.length}")
            GaussianVertex(modelledLength, 0.001).observe(dataPoint.length)
        }
        val model = BayesNet(helioStat.params.pivotPoint.x.connectedGraph)
        val optimiser = GradientOptimizer(model)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-15, 1e-15)
                )
        )
        return(helioStat.params.pivotPoint.getValue())
    }


    fun inferServoParams() : HelioStatParameters {
        val helioStat = HelioStat(ProbabilisticHelioStatParameters())
        for (dataPoint in this) {
            val sphericalNorm = helioStat.servoSignalToUnitSpherical(dataPoint.control)
            GaussianVertex(sphericalNorm.y, 0.001).observe(dataPoint.pitch)
            GaussianVertex(sphericalNorm.z, 0.001).observe(dataPoint.rotation)
        }
        val pmodel = BayesNet(helioStat.params.pitchParameters.m.connectedGraph)
        val poptimiser = GradientOptimizer(pmodel)
        poptimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-16, 1e-16)
                )
        )

        val rmodel = BayesNet(helioStat.params.rotationParameters.m.connectedGraph)
        val roptimiser = GradientOptimizer(rmodel)
        roptimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(
                        NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(1e-16, 1e-16)
                )
        )


        return helioStat.params.getValue()
    }


    fun calculateResiduals(params : HelioStatParameters) : ArrayList<Vector3D> {

        val forwardModel = HelioStat(ProbabilisticHelioStatParameters(params))
        var residual = ArrayList<Vector3D>(this.size)
        for(dataPoint in this) {
            val plane = forwardModel.computeHeliostatPlaneSpherical(
                    ConstantDoubleVertex(dataPoint.control.pitch.toDouble()),
                    ConstantDoubleVertex(dataPoint.control.rotation.toDouble())
            )
            val modelledPlane = plane.getValue()
//            println("modelled plane: $modelledPlane dataPoint: ${dataPoint.length}, ${dataPoint.pitch}, ${dataPoint.rotation} ")
            residual.add(modelledPlane.subtract(Vector3D(dataPoint.length, dataPoint.pitch, dataPoint.rotation)))
        }
        return residual
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

            val plane = forwardModel.computeHeliostatPlaneSpherical(
                    ConstantDoubleVertex(control.pitch.toDouble()),
                    ConstantDoubleVertex(control.rotation.toDouble())
            )
            plane.x.lazyEval()
            plane.y.lazyEval()
            plane.z.lazyEval()

            var modelledPlaneSpherical = plane.getValue()
            if(control.pitch < 0) {
                modelledPlaneSpherical = Geometry.erectToFlacid(modelledPlaneSpherical)
            }
            println("modelled plane: $modelledPlaneSpherical")
            this.add(DataPoint(modelledPlaneSpherical.x, modelledPlaneSpherical.y, modelledPlaneSpherical.z, control))
        }

    }
}