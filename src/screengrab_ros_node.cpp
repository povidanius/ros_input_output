#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Byte.h"
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

// X Server includes
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <linux/input.h>


void XImage2RosImage(XImage& ximage, Display& _xDisplay, Screen& _xScreen,
                     sensor_msgs::ImagePtr& im)
{
    XColor color;

    im->header.stamp = ros::Time::now();

    if (_xScreen.depths->depth == 24) {
        
        const int wd = ximage.width;
        const int ht = ximage.height;
        const int frame_size = wd * ht * 4;
        im->width = wd;
        im->height = ht;
        im->step = im->width * 4;

        im->encoding = sensor_msgs::image_encodings::BGRA8;
        im->data.resize(frame_size);
        memcpy(&im->data[0], ximage.data, frame_size);

    } else {
        Colormap colmap = DefaultColormap(&_xDisplay, DefaultScreen(&_xDisplay));
        for (int x = 0; x < ximage.width; x++) {
            for (int y = 0; y < ximage.height; y++) {
                color.pixel = XGetPixel(&ximage, x, y);
                XQueryColor(&_xDisplay, colmap, &color);
            }
        }
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "screengrab_ros_node");
    ros::NodeHandle nh, pn("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub;


    int update_rate;
    ros::param::param<int>("update_rate", update_rate, 15);
    ros::Rate loop_rate(update_rate);


    Display* display;
    Screen* screen;
    XImage* xImageSample;
    int screen_w, screen_h;

    display = XOpenDisplay(NULL);
    if (display == NULL) {
        ROS_ERROR_STREAM("bad display");
        return -1;
    }


    screen = DefaultScreenOfDisplay(display);
    if (screen == NULL) {
        ROS_ERROR_STREAM("bad screen");
        return -2;
    }



    Window wid = DefaultRootWindow( display );

    XWindowAttributes xwAttr;
    Status ret = XGetWindowAttributes( display, wid, &xwAttr );
    screen_w = xwAttr.width;
    screen_h = xwAttr.height;


    pub = it.advertise("/screen", 1);

    while (ros::ok())
    {
        sensor_msgs::ImagePtr im(new sensor_msgs::Image);

        int new_update_rate;
        ros::param::param<int>("update_rate", new_update_rate, 15);
        if (new_update_rate != update_rate) {
            loop_rate = ros::Rate(new_update_rate);
            update_rate = new_update_rate;
        }


        xImageSample = XGetImage(display, DefaultRootWindow(display),
                                 0, 0,screen_w , screen_h , AllPlanes, ZPixmap);

    
        if (xImageSample == NULL) {
            ROS_ERROR_STREAM("Error taking screenshot!");
            continue;
        }

                
        XImage2RosImage(*xImageSample, *display, *screen, im);
        pub.publish(im);
     
        XDestroyImage(xImageSample);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
