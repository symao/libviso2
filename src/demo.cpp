/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <viso_stereo.h>
#include <opencv2/opencv.hpp>

#include "vs_tictoc.h"
#include "vs_viz3d.h"

using namespace std;

int main (int argc, char** argv) {

    std::string sourceUri = "/home/symao/data/mynteye/20171107vins_outside/1/img.avi";
    cv::VideoCapture cap(sourceUri);
    if(!cap.isOpened())
    {
        printf("[ERROR] read source video failed. file '%s' not exists.\n", sourceUri.c_str());
        return 0;
    }
    int n_img = cap.get(CV_CAP_PROP_FRAME_COUNT);
  
    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h
    VisualOdometryStereo::parameters param;

    // calibration parameters for sequence 2010_03_09_drive_0019 
    param.calib.f  = 431.3179390163068; // focal length in pixels
    param.calib.cu = 404.4642372131348; // principal point (u-coordinate) in pixels
    param.calib.cv = 264.6879405975342; // principal point (v-coordinate) in pixels
    param.base     = 51.51650439372788/param.calib.f; // baseline in meters
    param.bucket.bucket_width = 75;
    param.bucket.bucket_height = 48;
    param.bucket.max_features = 2;
    param.ransac_iters     = 200;
    param.inlier_threshold = 1.5;
    param.reweighting      = true;

    // init visual odometry
    VisualOdometryStereo viso(param);

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    Matrix pose = Matrix::eye(4);

    Viz3dThread viz;
    std::vector<cv::Affine3f> traj;
    
    cv::Mat image;
    int cnt = 0;
    for(int i=0; i<n_img; i++){
        cap.read(image);
        if(i%2!=0) continue;  //skip half frames to reduce accumulate error

        if(image.channels()==3)
            cv::cvtColor(image,image,cv::COLOR_BGR2GRAY);
        int r = image.rows/2;
        cv::Mat imgl = image.rowRange(0,r);
        cv::Mat imgr = image.rowRange(r,r*2);
        int width = imgl.cols;
        int height = imgl.rows;

        tic("process");
        // compute visual odometry
        int32_t dims[] = {width,height,width};
        if (viso.process(imgl.data,imgr.data,dims)) {
            // on success, update current pose
            pose = pose * Matrix::inv(viso.getMotion());
        #if 1
            // output some statistics
            printf("#%d cost: %.1f ms\n", i, toc("process"));
            double num_matches = viso.getNumberOfMatches();
            double num_inliers = viso.getNumberOfInliers();
            cout << ", Matches: " << num_matches;
            cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
            cout << pose << endl << endl;
        #endif
        #if 1
            cv::Mat mat = (cv::Mat_<float>(4,4)<<pose.val[0][0],pose.val[0][1],pose.val[0][2],pose.val[0][3],
                                                pose.val[1][0],pose.val[1][1],pose.val[1][2],pose.val[1][3],
                                                pose.val[2][0],pose.val[2][1],pose.val[2][2],pose.val[2][3],
                                                pose.val[3][0],pose.val[3][1],pose.val[3][2],pose.val[3][3]);
            traj.push_back(cv::Affine3f(mat));
            viz.updateWidget("traj", cv::viz::WTrajectory(traj, 2, 1, cv::viz::Color::green()));
        #endif
        } else {
            cout << " ... failed!" << endl;
        }
        cnt++;
    }

    viz.updateWidget("traj", cv::viz::WTrajectory(traj, 2, 1, cv::viz::Color::green()));
    // output
    cout << "Demo complete! Exiting ..." << endl;
    getchar();
    // exit
    return 0;
}

