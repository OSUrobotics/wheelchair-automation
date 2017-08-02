#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";
int findBiggestContour(vector<vector<Point> >);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  int findBiggestContour(vector<vector<Point> > contours){
    int indexOfBiggestContour = -1;
    int sizeOfBiggestContour = 0;
    for (int i = 0; i < contours.size(); i++){
        if(contours[i].size() > sizeOfBiggestContour){
            sizeOfBiggestContour = contours[i].size();
            indexOfBiggestContour = i;
        }
    }
    return indexOfBiggestContour;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat downsampled;
    cv::Mat result; // segmentation result (4 possible values)
    cv::Mat bgModel;
    cv::Mat fgModel; // the models (internally used)
    cv::Mat blurred;
    cv::Mat hsv;
    cv::Mat bw;

    cv::pyrDown(cv_ptr->image, downsampled, cv::Size(cv_ptr->image.cols/2, cv_ptr->image.rows/2));

    // //Skin Detection
    // cv::blur( downsampled, blurred, cv::Size(3,3) );
    // cv::cvtColor(blurred, hsv, CV_BGR2HSV);
    // cv::inRange(hsv, cv::Scalar(0, 10, 60), cv::Scalar(20, 150, 255), bw);
    //
    // Mat canny_output;
    // vector<vector<Point> > contours;
    // vector<Vec4i> hierarchy;
    //
    // findContours( bw, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    // int s = findBiggestContour(contours);
    //
    // Mat drawing = Mat::zeros( downsampled.size(), CV_8UC1 );
    // drawContours( drawing, contours, s, Scalar(255), -1, 8, hierarchy, 0, Point() );
    //
    //
    // cv::imshow("Skin Detection", bw);
    // imshow("drw", drawing);

    //Test Mask Grabcut
    cv::Mat1b markers(cv_ptr->image.rows, cv_ptr->image.cols);
    // let's set all of them to possible background first
    markers.setTo(cv::GC_PR_BGD);
    // cut out a small area in the middle of the image
    int m_rows = 0.1 * cv_ptr->image.rows;
    int m_cols = 0.1 * cv_ptr->image.cols;
    // of course here you could also use cv::Rect() instead of cv::Range to select
    // the region of interest
    cv::Mat1b fg_seed = markers(cv::Range(cv_ptr->image.rows/2 - m_rows/2, cv_ptr->image.rows/2 + m_rows/2),
                                cv::Range(cv_ptr->image.cols/2 - m_cols/2, cv_ptr->image.cols/2 + m_cols/2));
    // mark it as foreground
    fg_seed.setTo(cv::GC_FGD);

    // select first 5 rows of the image as background
    cv::Mat1b bg_seed = markers(cv::Range(0, 5),cv::Range::all());
    bg_seed.setTo(cv::GC_BGD);

    cv::Mat bgd, fgd;

    // let's get all foreground and possible foreground pixels
    cv::Mat1b mask_fgpf = ( markers == cv::GC_FGD) | ( markers == cv::GC_PR_FGD);
    // and copy all the foreground-pixels to a temporary image
    cv::Mat3b tmp = cv::Mat3b::zeros(cv_ptr->image.rows, cv_ptr->image.cols);
    cv_ptr->image.copyTo(tmp, mask_fgpf);
    // show it
    cv::imshow("foreground", tmp);
    cv::waitKey(0);


    // cv::Rect rectangle(100,0,350,480);
    //
    // cv::grabCut(downsampled,result,rectangle,bgModel,fgModel,2,cv::GC_INIT_WITH_RECT); // use rectangle
    // cv::compare(result,cv::GC_PR_FGD,result,cv::CMP_EQ);
    // cv::Mat resultUp;
    // cv::pyrUp(result, resultUp, cv::Size(result.cols*2, result.rows*2));
    // cv::Mat foreground(cv_ptr->image.size(),CV_8UC3,cv::Scalar(255,255,255));
    // cv_ptr->image.copyTo(foreground,resultUp); // bg pixels not copied
    // cv::rectangle(foreground, rectangle, cv::Scalar(0,0,255),1);
    //
    // // Update GUI Window
    // cv::imshow("Masked Image", foreground);
    // cv::waitKey(3);
    //
    // // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
