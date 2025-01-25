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
package org.firstinspires.ftc.teamcode.base.config;

import java.io.IOException;
import java.io.InputStream;
import java.util.Objects;

public class Application {
    /**
     * This is the directory on the Robot Controller where metrics files are going to be saved
     * This should switch to a testing directory in a robot-less development environment
     */
    // public static String metricsDirName = "/storage/emulated/0/Android/data/com.qualcomm.ftcrobotcontroller/files/";
    public static String      metricsDirName = "C:/Temp";

    public static InputStream getResourceAsStream(String resourceName) throws IOException {
        InputStream input = Application.class.getResourceAsStream(resourceName);
        return input;
    }

    public static String getApplicationResourcesPath() {
        String   packageName          = Objects.requireNonNull(Application.class.getPackage()).getName();
        String[] packageDirs          = packageName.split("\\.");
        StringBuilder relativeDirUp   = new StringBuilder();
        StringBuilder relativeDirDown = new StringBuilder();
        for(String packageDir: packageDirs) {
            relativeDirUp.append("../");
            relativeDirDown.append(packageDir).append("/");
        }
        return relativeDirUp + "/" + relativeDirDown;
    }

    public static void main(String[] args) {
        String packageName = Objects.requireNonNull(Application.class.getPackage()).getName();
        System.out.println("Package is: " + packageName);
        System.out.println("Resources Path: " + getApplicationResourcesPath());
    }
}
