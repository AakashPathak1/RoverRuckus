package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, yellow.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 */
@TeleOp(name="ThreeBlockOpenCVYellow")
public class ThreeBlockOpenCVYellow extends OpMode {
    private ThreeBlockYellowVision yellowVision;
    @Override
    public void init() {
        yellowVision = new ThreeBlockYellowVision();
    // can replace with ActivityViewDisplay.getInstance() for fullscreen
        yellowVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        yellowVision.setShowCountours(false);
    // start the vision system
        yellowVision.enable();
}

    @Override
    public void loop() {
        // update the settings of the vision pipeline
        yellowVision.setShowCountours(true);

        // get a list of contours from the vision system
        List<MatOfPoint> contours = yellowVision.getContours();
        List<MatOfPoint> contourOne = yellowVision.getContourOne();
        boolean seesBlock;
        for (int i = 0; i < contourOne.size(); i++) {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e
            Rect boundingRect = Imgproc.boundingRect(contourOne.get(i));
            telemetry.addData("area", Imgproc.contourArea(contourOne.get(i)));
            telemetry.addData("x-coordinate", (boundingRect.x + boundingRect.width) / 2);
            telemetry.addData("y-coordinate", (boundingRect.y + boundingRect.height) / 2);
        }
        if (contourOne.size() > 0) {
            seesBlock = true;
        } else {
            seesBlock = false;
        }
        telemetry.addData("Sees Yellow Block", seesBlock);
    }

    public void stop() {
        // stop the vision system
        yellowVision.disable();
    }
}