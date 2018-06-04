import io.javalin.ApiBuilder.get
import io.javalin.ApiBuilder.post
import io.javalin.Javalin
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D


fun main(args : Array<String>) {

    val server = Server()

    server.start()

    var response = mutableMapOf("field1" to TestPayload("hello", 1234))

    val app = Javalin.create().apply {
        port(8080)
        exception(Exception::class.java) {e, ctx -> e.printStackTrace()}
        error(404) {ctx -> ctx.json("not found")}
    }.start()

    app.routes {
        get("/test/:id") { ctx ->
            response["field1"]?.id = ctx.param("id").orEmpty().toInt()
            response["field1"]?.name = ctx.queryParam("x")?:"no value"
            ctx.json(response)

        }

        post("/runModel") { ctx ->

            //response in JSON
        }

        post("navigate") { ctx ->
            val payload = ctx.bodyAsClass(NavigationQueryRead::class.java)
            ctx.status(201)
//            val navigator = HelioStatNavigator(payload[0].params)




            // get params, present control, bounds, point, solar vector
            // return new control params
        }

        post("navigatePointToPoint") {ctx ->
            // get params, present control, bounds, point, second point
            // return new control params

        }

        post("/calibrate") { ctx ->
            val payload = ctx.bodyAsClass(CalibrationDataReadAndConvert::class.java)
            ctx.status(201)



            var calib = HelioStatCalibration(payload)
            var params = calib.inferAllParams()

            // todo Get one with no dodgy data points and see the residual. Multiply by 5 and set that as a threshold.
            var avResidual = calib.calculateResiduals(params).sumByDouble(Vector3D::getNorm) / calib.size

            ctx.json(params)

            // receive whole caboodle in JSON format
            // return parameters
        }

    }
}
