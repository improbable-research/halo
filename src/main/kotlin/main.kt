fun main(args: Array<String>) {

    val server = Server()
    server.start()

}

//    val app = Javalin.create().apply {
//        port(8080)
//        exception(Exception::class.java) { e, ctx -> e.printStackTrace() }
//        error(404) { ctx -> ctx.json("not found") }
//    }.start()
//
//    app.routes {
//        get("/test") { ctx ->
//            ctx.json("Server running")
//        }
//
//        post("/setMirrorNormal") { ctx ->
//            val query = ctx.bodyAsClass(Query.SetNormal::class.java)
//            ctx.status(201)
//
//            val navigator = HelioStatNavigator(query.params)
//            val servoSetting = navigator.normalToServoSignal(query.mirrorNormal)
//            ctx.json(servoSetting)
//        }
//
//        post("/navigate") { ctx ->
//            val query = ctx.bodyAsClass(Query.Navigation::class.java)
//            ctx.status(201)
//
//            val navigator = HelioStatNavigator(query.params)
//            val servoSetting = navigator.computeServoSettingFromDirection(query.source, query.targetPoint, query.currentServoSetting)
//            ctx.json(servoSetting)
//        }
//
//        post("/navigatePointToPoint") { ctx ->
//            val query = ctx.bodyAsClass(Query.Navigation::class.java)
//            ctx.status(201)
//
//            val navigator = HelioStatNavigator(query.params)
//            val servoSetting = navigator.computeServoSettingFromPoint(query.source, query.targetPoint, query.currentServoSetting)
//            ctx.json(servoSetting)
//        }
//
//        post("/calibrate") { ctx ->
//            val payload = ctx.bodyAsClass(CalibrationDataReadAndConvert::class.java)
//            ctx.status(201)
//
//
//            var calib = HelioStatCalibration(payload)
//            var params = calib.inferAllParams()
//
//            // todo Get one with no dodgy data points and see the residual. Multiply by 5 and set that as a threshold.
//            var avResidual = calib.calculateResiduals(params).sumByDouble(Vector3D::getNorm) / calib.size
//
//            ctx.json(params)
//
//            // receive whole caboodle in JSON format
//            // return parameters
//        }
//
//    }
//}
