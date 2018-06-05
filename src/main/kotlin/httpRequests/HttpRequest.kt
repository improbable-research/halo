package httpRequests

import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.net.HttpURLConnection
import java.net.URL
import java.net.URLEncoder


class HttpRequest {

    companion object {
        fun get(url: String, requestParams: String): StringBuffer {

            val mURL = URL(url + "?" + requestParams)

            with(mURL.openConnection() as HttpURLConnection) {
                requestMethod = "GET"

                println("URL : $url")
                println("Response Code : $responseCode")

                BufferedReader(InputStreamReader(inputStream)).use {
                    val response = StringBuffer()

                    var inputLine = it.readLine()
                    while (inputLine != null) {
                        response.append(inputLine)
                        inputLine = it.readLine()
                    }
                    it.close()

                    return response
                }
            }
        }

        fun post(url: String, requestParams: String): StringBuffer {

            val mURL = URL(url)

            with(mURL.openConnection() as HttpURLConnection) {
                requestMethod = "POST"
                doOutput = true

                val wr = OutputStreamWriter(getOutputStream());
                wr.write(requestParams);
                wr.flush();

                println("URL : $url")
                println("Response Code : $responseCode")

                BufferedReader(InputStreamReader(inputStream)).use {
                    val response = StringBuffer()

                    var inputLine = it.readLine()
                    while (inputLine != null) {
                        response.append(inputLine)
                        inputLine = it.readLine()
                    }
                    it.close()

                    return response
                }
            }
        }

        fun makeRequestParam(key: String, value: String): String {
            return URLEncoder.encode(key, "UTF-8") + "=" + URLEncoder.encode(value, "UTF-8")
        }

        fun addRequestParamToString(key: String, value: String, string: String): String {
            return string + "&" + URLEncoder.encode(key, "UTF-8") + "=" + URLEncoder.encode(value, "UTF-8")
        }
    }
}
