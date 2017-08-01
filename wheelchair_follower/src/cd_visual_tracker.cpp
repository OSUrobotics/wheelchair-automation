#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

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

    cv::pyrDown(cv_ptr->image, downsampled, cv::Size(cv_ptr->image.cols/2, cv_ptr->image.rows/2));
    cv::Rect rectangle(120,1,380,479);
    cv::rectangle(cv_ptr->image, rectangle, cv::Scalar(0,0,255),1);

    cv::grabCut(downsampled,result,rectangle,bgModel,fgModel,1,cv::GC_INIT_WITH_RECT); // use rectangle
    cv::compare(result,cv::GC_PR_FGD,result,cv::CMP_EQ);
    cv::Mat resultUp;
    cv::pyrUp(result, resultUp, cv::Size(result.cols*2, result.rows*2));
    cv::Mat foreground(cv_ptr->image.size(),CV_8UC3,cv::Scalar(255,255,255));
    cv_ptr->image.copyTo(foreground,resultUp); // bg pixels not copied
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, foreground);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
