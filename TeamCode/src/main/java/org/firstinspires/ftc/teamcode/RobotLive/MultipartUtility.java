package org.firstinspires.ftc.teamcode.RobotLive;

import android.graphics.Bitmap;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

public class MultipartUtility {
    private final String boundary;
    private static final String LINE_FEED = "\r\n";
    private HttpURLConnection httpConn;
    private String charset;
    private OutputStream outputStream;
    private PrintWriter writer;

    public MultipartUtility(String requestURL, String charset) throws IOException {
        this.charset = charset;
        this.boundary = "===" + System.currentTimeMillis() + "===";
        URL url = new URL(requestURL);
        this.httpConn = (HttpURLConnection)url.openConnection();
        this.httpConn.setUseCaches(false);
        this.httpConn.setDoOutput(true);
        this.httpConn.setDoInput(true);
        this.httpConn.setRequestProperty("Content-Type", "multipart/form-data; boundary=" + this.boundary);
        this.outputStream = this.httpConn.getOutputStream();
        this.writer = new PrintWriter(new OutputStreamWriter(this.outputStream, charset), true);
    }

    public void addFormField(String name, String value) {
        this.writer.append("--" + this.boundary).append("\r\n");
        this.writer.append("Content-Disposition: form-data; name=\"" + name + "\"").append("\r\n");
        this.writer.append("Content-Type: text/plain; charset=" + this.charset).append("\r\n");
        this.writer.append("\r\n");
        this.writer.append(value).append("\r\n");
        this.writer.flush();
    }

    public void addFilePart(String fieldName, File uploadFile) throws IOException {
        this.addFilePart(fieldName, uploadFile.getName(), new FileInputStream(uploadFile));
    }

    public void addFilePart(String fieldName, String fileName, InputStream inputStream) throws IOException {
        this.writer.append("--" + this.boundary).append("\r\n");
        this.writer.append("Content-Disposition: form-data; name=\"" + fieldName + "\"; filename=\"" + fileName + "\"").append("\r\n");
        this.writer.append("Content-Type: image/jpeg").append("\r\n");// + URLConnection.guessContentTypeFromName(fileName)
        this.writer.append("Content-Transfer-Encoding: binary").append("\r\n");
        this.writer.append("\r\n");
        this.writer.flush();
        byte[] buffer = new byte[4096];
        boolean var5 = true;

        int bytesRead;
        while((bytesRead = inputStream.read(buffer)) != -1) {
            this.outputStream.write(buffer, 0, bytesRead);
        }

        this.outputStream.flush();
        inputStream.close();
        this.writer.append("\r\n");
        this.writer.flush();
    }

    public void addBitmap(String fieldName, Bitmap bitmap) throws IOException {
        this.writer.append("--" + this.boundary).append("\r\n");
        this.writer.append("Content-Disposition: form-data; name=\"" + fieldName + "\"; filename=\"" + fieldName + ".jpg\"").append("\r\n");
        //this.writer.append("Content-Type: image/jpeg").append("\r\n");// + URLConnection.guessContentTypeFromName(fileName)
        this.writer.append("Content-Type: image/jpeg").append("\r\n");//URLConnection.guessContentTypeFromName(fieldName + ".jpg")).append("\r\n");
        this.writer.append("Content-Transfer-Encoding: binary").append("\r\n");
        this.writer.append("\r\n");
        this.writer.flush();
//        byte[] buffer = new byte[4096];
//        boolean var5 = true;


//        int width = bitmap.getWidth();
//        int height = bitmap.getHeight();
//
//        int size = bitmap.getRowBytes() * bitmap.getHeight();
//        ByteBuffer byteBuffer = ByteBuffer.allocate(size);
//        bitmap.copyPixelsToBuffer(byteBuffer);
// this.outputStream.write(byteBuffer.array());

//        ByteArrayOutputStream bos = new ByteArrayOutputStream();
//        bitmap.compress(Bitmap.CompressFormat.JPEG, 90 /*ignored for PNG*/, bos);
//        byte[] bitmapdata = bos.toByteArray();
        bitmap.compress(Bitmap.CompressFormat.JPEG, 90, this.outputStream);
        //this.outputStream.write(bitmapdata);
        this.outputStream.flush();
        this.writer.append("\r\n");
        this.writer.flush();
    }

    public void addHeaderField(String name, String value) {
        this.writer.append(name + ": " + value).append("\r\n");
        this.writer.flush();
    }

    public List<String> finish() throws IOException {
        List<String> response = new ArrayList();
        this.writer.append("\r\n").flush();
        this.writer.append("--" + this.boundary + "--").append("\r\n");
        this.writer.close();
        int status = this.httpConn.getResponseCode();
        if (status != 200) {
            throw new IOException("Server returned non-OK status: " + status);
        } else {
            BufferedReader reader = new BufferedReader(new InputStreamReader(this.httpConn.getInputStream()));
            String line = null;

            while((line = reader.readLine()) != null) {
                response.add(line);
            }

            reader.close();
            this.httpConn.disconnect();
            return response;
        }
    }
}
