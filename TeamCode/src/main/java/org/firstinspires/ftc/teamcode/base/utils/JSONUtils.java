/*
 * Copyright (c) 2025 Murad Nayal
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name Murad Nayal nor the names of contributors to this material may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.base.utils;

import androidx.annotation.NonNull;

import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.Map;

import com.google.gson.Gson;

import org.firstinspires.ftc.teamcode.base.config.Application;

public class JSONUtils {
    public static <T> T parseJSON(Reader input, Class<T> contentsClass) {
        Gson gson = new Gson();
        return gson.fromJson(input, contentsClass);
    }

    public static class JsonTest {
        public static class MotorTest {
            public String k1;
            public int    k2;
            @NonNull
            public String toString() {
                var sb = new StringBuilder();
                sb.append("MotorTest\n");
                sb.append("  k1=").append(k1).append("\n");
                sb.append("  k2=").append(k2).append("\n");
                return sb.toString();
            }
        }
        public static class ServoTest {
            public String k1;
            public int    k2;
            @NonNull
            public String toString() {
                var sb = new StringBuilder();
                sb.append("ServoTest\n");
                sb.append("  k1=").append(k1).append("\n");
                sb.append("  k2=").append(k2).append("\n");
                return sb.toString();
            }
        }

        public String                 title;
        public int                    version;
        public Map<String, MotorTest> motors;
        public Map<String, ServoTest> servos;

        @NonNull
        public String toString() {
            var sb = new StringBuilder();
            sb.append("title=").append(title).append("\n");
            sb.append("version=").append(version).append("\n");
            sb.append("motors").append("\n");
            for(var entry: motors.entrySet())
                sb.append(entry.getKey()).append("=").append(entry.getValue().toString());
            sb.append("servos");
            for(var entry: servos.entrySet())
                sb.append(entry.getKey()).append("=").append(entry.getValue().toString());
            return sb.toString();
        }
    }

    public static void main(String[] args) {
        InputStream input    = Application.getResourceAsStream("Test1.json");
        Reader      reader   = new InputStreamReader(input);
        JsonTest    contents = parseJSON(reader, JsonTest.class);
        System.out.println(contents.toString());
    }
}
