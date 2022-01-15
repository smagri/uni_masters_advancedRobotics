// Author: Simone Magri
// Subject: Advanced Robotics Spring Semester at UTS
//
// Feature Based MonoSLAM
//
// ***************************************************************************


#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/xfeatures2d/nonfree.hpp"

//#include "ParallaxBA/ParallaxBAImp.h"
#include "ParallaxBA/config.h"

using namespace std;
using namespace cv;
using namespace xfeatures2d; // opencv3.0.0gold

void readme();


/** @function main */
int main( int argc, char** argv ){

  if( argc != 3 ){
    readme();
    return -1;
  }


  Mat img1 = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  Mat img2 = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);

  if( !img1.data || !img2.data ){
    std::cout<< " --(!) Error reading images " << std::endl; 
    return -1;
  }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400; // default value

  // Needed opencv2.4.8 SURF ported to opencv3.0.0gold SURF api
  //SurfFeatureDetector detector(minHessian);
  //
  // opencv3.0.0gold:
  //cv::Ptr<Feature2D> features = xfeatures2d::SURF::create(minHessian);
  cv::Ptr<Feature2D> features = SURF::create(minHessian);


  // Detection of features(keypoints) via the SURF algorithm.
  std::vector<KeyPoint> keypointsImg1, keypointsImg2;
  features->detect(img1, keypointsImg1);
  features->detect(img2, keypointsImg2);

  //-- Step 2: Calculate descriptors (feature vectors)
  //SurfDescriptorExtractor extractor;
  Mat descriptorsImg1, descriptorsImg2;
  features->compute(img1, keypointsImg1, descriptorsImg1);
  features->compute(img2, keypointsImg2, descriptorsImg2);

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector<DMatch> matches;
  matcher.match(descriptorsImg1, descriptorsImg2, matches);


  //-- Quick calculation of max and min distances between keypoints.
  //-- Sanity check to make sure we don't have img1=img2?
  double max_dist = 0; double min_dist = 100;
  //
  for( int i = 0; i < descriptorsImg1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );


  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector<DMatch> goodMatches;
  //
  for(int i = 0; i < descriptorsImg1.rows; i++){
    if(matches[i].distance < 3*min_dist){
      goodMatches.push_back(matches[i]);
    }
  }

  Mat outputImage;
  drawMatches(img1, keypointsImg1, img2, keypointsImg2,
	      goodMatches, //matches between image1 and image2
	      outputImage, //output image with flags(howto draw image) as input
	      Scalar::all(-1), Scalar::all(-1), //use random colours
	      vector<char>(), //mask for which matches are drawn,is empty=>all
	      DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS //noSingleKeypointsDrawn
	      );

  //-- Localize to determine essential matrix and relative camera pose.
  std::vector<Point2f> image1; // pixels x=(u,v)=(x,y)
  std::vector<Point2f> image2; // pixels x'=(u',v')==(x',y')
  //
  /* example of use:
     typedef Point_<float> Point2f;
     typedef Point_<double> Point2d;
     
     Point2f a(0.3f, 0.f), b(0.f, 0.4f);
     Point pt = (a + b)*10.f;
     cout << pt.x << ", " << pt.y << endl;

  */
  //
  for(int i = 0; i < goodMatches.size(); i++){
    // Use the keypoints from the good matches and determine their
    // corresponding (x,y) coordinates.  Put the (x,y) coordinates
    // into the image1 and image2 vectors.
    image1.push_back(keypointsImg1[goodMatches[i].queryIdx].pt);
    image2.push_back(keypointsImg2[goodMatches[i].trainIdx].pt);
  }

  cout<<"fbmslam: "<<goodMatches.size()<<" good matches found\n";

  // Draw all good feature matches, thus includes all inliers and outliers.
  string goodMatchesWin = "Good Matches";
  namedWindow(goodMatchesWin,
	      (CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED) );
  resizeWindow(goodMatchesWin, 1800, 800);
  //-- Show detected matches
  imshow(goodMatchesWin, outputImage);


  // Calculate the  fundamental matrix and  consequently determine the
  // inliers amongst the goodMatches.
  //
  // FM_RANSAC applies the RANSAC algorithm to the image points, it is
  // applicable when the number of points is >= 8.
  //
  // param1=1-3 is used for the ransac algorithm, the maximum distance
  // from a  point to  the epipolar line  in pixels, beyond  which the
  // point is  considered an outlier  and is not used  for calculating
  // the fundamental matrix F.  Default value=3.
  //
  // Note, for inliers the epipolar  constraint holds: xTransposed*F*x'=0; 
  //
  // where img1=x(u,v) and img2=x'(u',v') and F=fundamental matrix
  //
  // param2=0.99 is  used for the  ransac algorith, and  specifies the
  // desirable  level of  confidence(probability)  that the  estimated
  // matrix is correct.  Default value is=0.99.
  //
  //std::vector<uchar> mask;
/*  Mat mask; // mask = [mask.rows, 1] matrix
  Mat F =
    findFundamentalMat(image1, image2,// Img1 & Img2 projected points(x,y)
		       FM_RANSAC,// use ransac algorithm get only inliers in F
		       3, // default value
		       0.99, // default value for
		       mask);
*/

  // Determine  essential  matrix  E,  will  also  be  used  later  in
  // recoverPose() to determine the [R  | t] matrix.  P = K[R|t] where
  // P is  the projection matrix, R  is the rotation matrix,  t is the
  // translation matrix;  it maps any  point in eucledian 3D  space to
  // the image plane of the camera so x(u,v pixels)=PX(x,y,z);
  //
  //double focalLength = 1.0; // default value
  //Point2d principalPoint(0.0, 0.0); // default value
  //
  // Given the intrinsic calibration matrix K in calSBA.txt the focal
  // length are principal point are defined as follows.  Non-square
  // pixels are assumend(due to spherical camera lens/ccd).
  //
  //
  // K= 923.5295	0		507.2222
  //  = 0		922.2418	383.5822
  //  = 0		0		1
  Mat K = (Mat_<double>(3,3) <<
	   923.5295, 0.0, 507.2222,
	   0.0, 922.2418, 383.5822,
	   0.0, 0.0, 1.0);
  // Accessing K elements eg:
  /* Mat_<double> M(20,20);
     for(int i = 0; i < M.rows; i++)
       for(int j = 0; j < M.cols; j++)
         M(i,j) = 1./(i+j+1);
     Mat E, V;
     eigen(M,E,V);
     cout << E.at<double>(0,0)/E.at<double>(M.rows-1,0);
  */

  // This appears to be the Malaga camera matrix, cal170.txt
  double ax = 923.5295;
  double ay = 922.2418;
  double focalLength = sqrt( pow(ax,2) + pow(ay,2) );
  //
  double ppx = 507.2222;
  double ppy = 383.5822;
  Point2d principalPoint(ppx, ppy);
  //
  double threshold = 2.0; // 0.5 and 1.0 wrk too 3.0 for findFundamental() 
  double probability = 0.99;
  Mat mask; // mask = [mask.rows, 1] matrix
  Mat E; // essential matrix
  //
  E = findEssentialMat(image1, image2, focalLength, principalPoint,
		       FM_RANSAC, probability, threshold, mask);
  

  // Determine the inliers between the keypoint matches(ie withing goodMatches).
  std::vector<DMatch> inlierMatches;
  std::vector<DMatch> outlierMatches;
  for (int i=0; i<mask.rows; i++){
    if ( ((int)mask.at<uchar>(i, 0)) == 1 ){
      // This match is an inlier, determined via the RANSAC algorithm
      // within   fundFundamentalMat().   Thus,  these   matches  must
      // satisfy the epipolar constraint.
      inlierMatches.push_back(goodMatches[i]);
    }
    else{
      outlierMatches.push_back(goodMatches[i]);
    }
  }
  cout<<"fbmslam: "<<inlierMatches.size()<<" inlier matches found\n";

  // Only draw inliers in the second window=inliersWin.
  // 
  drawMatches(img1, keypointsImg1, img2, keypointsImg2,
	      inlierMatches, outputImage,
	      Scalar::all(-1), Scalar::all(-1),
	      vector<char>(), 
	      DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  //
  string inliersWin = "Only Inliers";
  namedWindow(inliersWin,
	      (CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED) );
  resizeWindow(inliersWin, 1800, 800);
  imshow(inliersWin, outputImage);
  

  // Only draw outliers in the third window=oulierWin
  //
  // to specify single colour: 
  //   CV_RGB(255,0,0) instead of Scalar::all(-1)
  //   Scalar(255, 255, 255)
  drawMatches(img1, keypointsImg1, img2, keypointsImg2,
	      outlierMatches, outputImage,
	      Scalar::all(-1), Scalar::all(-1),
	      vector<char>(), 
	      DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  //
  string outliersWin = "Only Outliers";
  namedWindow(outliersWin,
	      (CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED) );
  resizeWindow(outliersWin, 1800, 800);
  imshow(outliersWin, outputImage);  
  

  
  //  Recover  relative  camera   rotation  and  translation  from  an
  //  estimated essential  matrix and the corresponding  points in two
  //  images, using  cheirality check.  Returns the number  of inliers
  //  which pass the check.
  //
  //  Only pass inliers into this function.
  Mat R; // recovered relative rotation matrix, change in where your
	 // pointing/orienting
  Mat t; // recovered relative translation matrix, change in x,y,z
  //
  Mat relativeCameraPose;
  // Focal length and principal point assumed the same for all/both images.
  //
  // Inliner/Outliner mask for image1 & image2 points.  This refers to
  // the Essential matrix used here, and calculated earlier.  However,
  // only the  inliers will be  used to recover relative  pose between
  // the camera images of image1/2.
  //
  // This    function   decomposes    an   essential    matrix   using
  // decomposeEssentialMat()   and   then   verifies   possible   pose
  // hypotheses  by  doing  cheirality  check.  The  cheirality  check
  // basically  means  that the  triangulated  3D  points should  have
  // positive depth. Some details can be found in [Nister03].
  //
  relativeCameraPose
    = recoverPose(E, image1, image2, R, t, focalLength, principalPoint, mask);

  cout<<"fbmslam: Relative Camera Pose:\n";

  cout<<"\nfbmslam: Relative Rotation Matrix R =\n"<<R<<"\n\n";
  cout<<"fbmslam: Relative Translation Matrix t =\n"<<t<<"\n\n";
  //  cout<<"fbmslam: Relative Camera Pose =\n"<<relativeCameraPose<<"\n\n";
  
  // Calculate the projection matrix
  //Mat P = K * [R | t];
  //
  Mat P = K * R;
  cout<<"fbmslam: Camera Calibration Matrix K =\n"<<K<<"\n\n";
  cout<<"fbmslam: Projection Matrix P =\n"<<P<<"\n\n";

  // Howto access Mat elements for R and t:
  //
  // wrks: cout<<"fbmslam: R(1,1)="<<R.at<double>(1,1)<<endl;
  // no wrks: cout<<"fbmslam: R(1,1)="<<R[1][1]<<endl;
  // wrks, fastest method:
  //	double *ptrRowR =R.ptr<double>(1);
  //	cout<<"fbmslam: R(1,1)="<<ptrRowR[1]<<endl;
  //
  // Example code, slowest but most convienent method:
  //	Mat M(100, 100, CV_64F);
  //	cout << M.at<double>(0,0);
  //int rMatType = Mat::type(R);
  //cout<<"fbmslam: R matrix type = "<<rMatType;
  //  cout<<"fbmslam: dbg: c++ = R(1,1)="<<R.at<double>(1,1)<<endl;
  //  fprintf(stderr, "fbmslam: dbg: c = R(1,1)=%.15f\n", R.at<double>(1,1));


  // Euler Angle, the rotation angle along the x,y,z axis, determines
  // camera orintation. Translation matrix t determines camera centre.
  //
  // note: atan2(y,x)=the principal arc tangent of (y/x) in radians,
  //	   multiply by 180/pi to convert to degrees.
  //
  double thetaX = atan2(R.at<double>(3,2), R.at<double>(3,3));
  double thetaY =
    atan2( (-1 * R.at<double>(3,1)),
	   sqrt(pow(R.at<double>(3,2),2) + pow(R.at<double>(3,3),2)) );
  double thetaZ = atan2(R.at<double>(2,1), R.at<double>(1,1));
  cout<<"fbmslam: Euler Angles in radians:\n";
  cout<<"fbmslam: thetaX ="<< thetaX <<endl;
  cout<<"fbmslam: thetaY ="<< thetaY <<endl;
  cout<<"fbmslam: thetaZ ="<< thetaZ <<endl;



  // Bundle Adjustment via the ParallexBA algorithm
  //
  // Command line run command:
  //
  // ParallaxBA -cam Cam170.txt -calib cal170.txt -fea Feature170.txt	\
  // -solve LM -report report.txt
  //
/*  CParallaxBA ba;
  //ba.pba_run(Ca170.txt, Feature170, cal170.txt, LM, report.txt);
  ba.pba_run("-cam Cam170.txt", "-fea Feature170.txt", "-calib cal170.txt",
	     "-solve LM", "-report report.txt");

*/

			 

  waitKey(0);
  return 0;

}



void readme(){
  /** @function readme */
  
  std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl;
}
