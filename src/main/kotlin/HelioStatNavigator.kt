import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.UniformVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.optim.SimpleValueChecker
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer
import kotlin.math.roundToInt

class HelioStatNavigator {

    private val convergenceThreshold = 1e-13

    class NavigationQuery (var params: HelioStatParameters, var presentControl: ServoSetting,
                           var targetPoint: Vector3D, var source: Vector3D) {}

    val servoPitchRange : DoubleVertex
    val servoRotationRange : DoubleVertex
    val targetDistance : DoubleVertex
    val model : HelioStat

    constructor(params : HelioStatParameters) {
        servoPitchRange = UniformVertex(-10000.0, 8096.0)
        servoRotationRange = UniformVertex(-10000.0, 8096.0)
        targetDistance = UniformVertex(-100.0, 100.0)
        model = HelioStat(ProbabilisticHelioStatParameters(params))
        servoPitchRange.value = 0.0
        servoRotationRange.value = 0.0
        targetDistance.value = 5.0
    }

    fun normalToServoSignal(mirrorNormal : Vector3D) : ServoSetting {
        val targetObservationNoise = Vector3D(0.001, 0.001, 0.001)
        val probNormal = model.computeHeliostatNormal(servoPitchRange, servoRotationRange)
        probNormal.noisyObserve(mirrorNormal, targetObservationNoise)

        var sphericalNormal = Geometry.cartesianToSpherical(mirrorNormal)
        if(sphericalNormal.z < -Math.PI/2.0) sphericalNormal = Vector3D(sphericalNormal.x, sphericalNormal.y, sphericalNormal.z + 2.0*Math.PI)

        val initialGuessPitch = (sphericalNormal.y - model.params.pitchParameters.c.value)/model.params.pitchParameters.m.value
        val initialGuessRotation = (sphericalNormal.z - model.params.rotationParameters.c.value)/model.params.rotationParameters.m.value

        servoPitchRange.value = initialGuessPitch
        servoRotationRange.value = initialGuessRotation

        val net = BayesNet((servoRotationRange + servoPitchRange).connectedGraph)

        val optimiser = GradientOptimizer(net)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(convergenceThreshold, convergenceThreshold)))

        return ServoSetting(servoRotationRange.value.roundToInt(), servoPitchRange.value.roundToInt())
    }

    private fun computeServoSetting(probabilisticTargetPoint: ProbabilisticVector3D,
                                    desiredTargetPoint: Vector3D): ServoSetting {

        val targetObservationNoise = Vector3D(0.001, 0.001, 0.001)
        probabilisticTargetPoint.noisyObserve(desiredTargetPoint, targetObservationNoise)

        val net = BayesNet((servoRotationRange + servoPitchRange).connectedGraph)

        val optimiser = GradientOptimizer(net)
        optimiser.maxAPosteriori(10000,
                NonLinearConjugateGradientOptimizer(NonLinearConjugateGradientOptimizer.Formula.POLAK_RIBIERE,
                        SimpleValueChecker(convergenceThreshold, convergenceThreshold)))

        println("opitimised target distance is ${targetDistance.value}")

        return ServoSetting(servoRotationRange.value.roundToInt(), servoPitchRange.value.roundToInt())
    }

    fun computeServoSettingFromDirection(incomingSunDirection: Vector3D,
                                         desiredTargetPoint: Vector3D, currentSetting : ServoSetting): ServoSetting {
        val inRay = incomingSunDirection.normalize()
        val outRay = (desiredTargetPoint.subtract(model.params.pivotPoint.getValue())).normalize()
        val norm = outRay.subtract(inRay).normalize()

        val firstGuess = normalToServoSignal(norm)
        targetDistance.value = desiredTargetPoint.subtract(model.params.pivotPoint.getValue()).dotProduct(outRay)/incomingSunDirection.norm

        if(currentSetting.rotation < 0 || currentSetting.pitch < 0) {
            servoPitchRange.value = firstGuess.pitch.toDouble()
            servoRotationRange.value = firstGuess.rotation.toDouble()
        } else {
            servoPitchRange.value = currentSetting.pitch.toDouble()
            servoRotationRange.value = currentSetting.rotation.toDouble()
        }

        //       println("first guess is ${firstGuess.pitch} ${firstGuess.rotation} ${targetDistance.value}")

        val probabilisticTargetPoint = model.computeTargetFromSourceDirection(servoPitchRange, servoRotationRange,
                                                                              ProbabilisticVector3D(incomingSunDirection), targetDistance)
        return computeServoSetting(probabilisticTargetPoint, desiredTargetPoint)
    }

    fun computeServoSettingFromPoint(sourcePoint: Vector3D, desiredTargetPoint: Vector3D, currentSetting : ServoSetting): ServoSetting {
        val inRay = (model.params.pivotPoint.getValue().subtract(sourcePoint)).normalize()
        val outRay = (desiredTargetPoint.subtract(model.params.pivotPoint.getValue())).normalize()
        val norm = outRay.subtract(inRay).normalize()

        val firstGuess = normalToServoSignal(norm)
        targetDistance.value = desiredTargetPoint.subtract(model.params.pivotPoint.getValue()).dotProduct(outRay)/(model.params.pivotPoint.getValue().subtract(sourcePoint)).norm

        if(currentSetting.rotation < 0 || currentSetting.pitch < 0) {
            servoPitchRange.value = firstGuess.pitch.toDouble()
            servoRotationRange.value = firstGuess.rotation.toDouble()
        } else {
            servoPitchRange.value = currentSetting.pitch.toDouble()
            servoRotationRange.value = currentSetting.rotation.toDouble()
        }

//        println("first guess is ${firstGuess.pitch} ${firstGuess.rotation} ${targetDistance.value}")

        val probabilisticTargetPoint = model.computeTargetFromSourcePoint(servoPitchRange, servoRotationRange,
                                                                          sourcePoint, targetDistance)
        return computeServoSetting(probabilisticTargetPoint, desiredTargetPoint)
    }
}
