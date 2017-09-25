// copyright Lucas Walter November 2013

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
#include "ros_input_output/gui_event.h"

// X Server includes
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <linux/input.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Xutil.h>
#include <X11/Shell.h>



//#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xproto.h>
#include <X11/extensions/record.h>

#include <xosd.h>


#include <fcntl.h>

#define LEFT_BUTTON 1
#define RIGHT_BUTTON 3


/* for this struct, refer to libxnee */
typedef union {
    unsigned char type ;
    xEvent event ;
    xResourceReq req ;
    xGenericReply reply ;
    xError error ;
    xConnSetupPrefix setup;
} XRecordDatum;



ros::Publisher mouse_pub;

Display *d0, *d1;

void mouseClick(Display *display, int button) {
    // Create and setting up the event
    XEvent event;
    memset(&event, 0, sizeof(event));
    event.xbutton.button = button;
    event.xbutton.same_screen = True;
    event.xbutton.subwindow = DefaultRootWindow(display);
    while(event.xbutton.subwindow) {
        event.xbutton.window = event.xbutton.subwindow;
        XQueryPointer(display, event.xbutton.window,
                      &event.xbutton.root, &event.xbutton.subwindow,
                      &event.xbutton.x_root, &event.xbutton.y_root,
                      &event.xbutton.x, &event.xbutton.y,
                      &event.xbutton.state);
    }
    // Press
    event.type = ButtonPress;
    if(XSendEvent(display, PointerWindow, True, ButtonPressMask, &event) == 0)
        fprintf(stderr, "Error sending the click event!\n");
    XFlush(display);
}


void mouseRelease(Display *display, int button) {
    // Create and setting up the event
    XEvent event;
    memset(&event, 0, sizeof(event));
    event.xbutton.button = button;
    event.xbutton.same_screen = True;
    event.xbutton.subwindow = DefaultRootWindow(display);
    while(event.xbutton.subwindow) {
        event.xbutton.window = event.xbutton.subwindow;
        XQueryPointer(display, event.xbutton.window,
                      &event.xbutton.root, &event.xbutton.subwindow,
                      &event.xbutton.x_root, &event.xbutton.y_root,
                      &event.xbutton.x, &event.xbutton.y,
                      &event.xbutton.state);
    }
    // Release
    event.type = ButtonRelease;
    if(XSendEvent(display, PointerWindow, True, ButtonReleaseMask, &event) == 0)
        fprintf(stderr, "Error sending the click event!\n");
    XFlush(display);
}


void mouseClickRelease(Display *display, int button) {
    // Create and setting up the event
    XEvent event;
    memset(&event, 0, sizeof(event));
    event.xbutton.button = button;
    event.xbutton.same_screen = True;
    event.xbutton.subwindow = DefaultRootWindow(display);
    while(event.xbutton.subwindow) {
        event.xbutton.window = event.xbutton.subwindow;
        XQueryPointer(display, event.xbutton.window,
                      &event.xbutton.root, &event.xbutton.subwindow,
                      &event.xbutton.x_root, &event.xbutton.y_root,
                      &event.xbutton.x, &event.xbutton.y,
                      &event.xbutton.state);
    }
    // Press
    event.type = ButtonPress;
    if(XSendEvent(display, PointerWindow, True, ButtonPressMask, &event) == 0)
        fprintf(stderr, "Error sending the click event!\n");
    XFlush(display);
    // Release
    event.type = ButtonRelease;
    if(XSendEvent(display, PointerWindow, True, ButtonReleaseMask, &event) == 0)
        fprintf(stderr, "Error sending the click event!\n");
    XFlush(display);
}


void mouseMove(Display *display, int x, int y) {
    Window root_window = XRootWindow(display, 0);
    XSelectInput(display, root_window, KeyReleaseMask);
    XWarpPointer(display, None, root_window, 0,0,0,0, x, y);
    XSync(display, False);
    XFlush(display);
}


void event_callback(XPointer priv, XRecordInterceptData *hook)
{
    /* FIXME: we need use XQueryPointer to get the first location */
    ros_input_output::gui_event gui_event;

    static int cur_x = 0;
    static int cur_y = 0;
    if (hook->category != XRecordFromServer) {
        XRecordFreeData (hook);
        return;
    }
    XRecordDatum *data = (XRecordDatum*) hook->data;
    int event_type = data->type;
    BYTE btncode, keycode;
    btncode = keycode = data->event.u.u.detail;
    int rootx = data->event.u.keyButtonPointer.rootX;
    int rooty = data->event.u.keyButtonPointer.rootY;


    switch (event_type) {
    case KeyPress:
        /* if escape is pressed, stop the loop and clean up, then exit */
        // if (keycode == 9) stop = 1;
        /* Note: you should not use data_disp to do normal X operations !!!*/
        //printf ("KeyPress: \t%s\n", XKeysymToString(XKeycodeToKeysym(d0, keycode, 0)));



        break;
    case KeyRelease:
        //printf ("KeyRelease: \t%s\n", XKeysymToString(XKeycodeToKeysym(d0, keycode, 0)));
        break;
    case ButtonPress:
        //printf ("ButtonPress: /t%d, rootX=%d, rootY=%d", btncode, cur_x, cur_y);

        gui_event.header.stamp = ros::Time::now();
        gui_event.type = ros_input_output::gui_event::MOUSE_EVENT_PRESS;

        gui_event.mouse_x = cur_x;
        gui_event.mouse_y = cur_y;
        if (btncode == LEFT_BUTTON)
            gui_event.mouse_left = 1;
        else if (btncode == RIGHT_BUTTON)
            gui_event.mouse_right = 1;

        gui_event.btncode = btncode;

        mouse_pub.publish(gui_event);

        break;
    case ButtonRelease:
        //printf ("ButtonRelease: /t%d, rootX=%d, rootY=%d", btncode, cur_x, cur_y);

        gui_event.header.stamp = ros::Time::now();
        gui_event.type = ros_input_output::gui_event::MOUSE_EVENT_RELEASE;
        gui_event.mouse_x = cur_x;
        gui_event.mouse_y = cur_y;
        if (btncode == LEFT_BUTTON)
            gui_event.mouse_left = 0;
        else if (btncode == RIGHT_BUTTON)
            gui_event.mouse_right = 0;

        gui_event.btncode = btncode;
        mouse_pub.publish(gui_event);


        break;
    case MotionNotify:
        //printf ("MouseMove: /trootX=%d, rootY=%d",rootx, rooty);

        gui_event.header.stamp = ros::Time::now();
        gui_event.type = ros_input_output::gui_event::MOUSE_EVENT_MOVE;

        cur_x = rootx;
        cur_y = rooty;

        gui_event.mouse_x = cur_x;
        gui_event.mouse_y = cur_y;

        mouse_pub.publish(gui_event);

        break;
    case CreateNotify:
        break;
    case DestroyNotify:
        break;
    case NoExpose:
        break;
    case Expose:
        break;
    default:
        break;
    }
    //printf (", time=%d\n", time);

    XRecordFreeData (hook);
}


void mouseCallback(const ros_input_output::gui_event::ConstPtr& msg)
{

    switch (msg->type)
    {
    case ros_input_output::gui_event::MOUSE_EVENT_MOVE:
        ROS_INFO("X: Moving mouse %d %d", msg->mouse_x, msg->mouse_y);
        mouseMove(d0, msg->mouse_x, msg->mouse_y);
        break;
    case ros_input_output::gui_event::MOUSE_EVENT_PRESS:

        ROS_INFO("Moving mouse (press) %d %d", msg->mouse_x, msg->mouse_y);
        mouseMove(d0, msg->mouse_x, msg->mouse_y);

        if (msg->mouse_left == 1)
        {
            ROS_INFO("Left click");
            mouseClick(d0, LEFT_BUTTON);
        }
        else if (msg->mouse_right == 1)
        {
            ROS_INFO("Right click");
            mouseClick(d0, RIGHT_BUTTON);
        }
        break;
    case ros_input_output::gui_event::MOUSE_EVENT_RELEASE:
        ROS_INFO("Moving mouse (release) %d %d", msg->mouse_x, msg->mouse_y);
        mouseMove(d0, msg->mouse_x, msg->mouse_y);

        if (msg->mouse_left == 0)
        {
            ROS_INFO("Left release");
            mouseRelease(d0, LEFT_BUTTON);
        }
        else if (msg->mouse_right == 0)
        {
            ROS_INFO("Right release");
            mouseRelease(d0, RIGHT_BUTTON);
        }
        break;
    default:
        break;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mouse_ros_node");
    ros::NodeHandle nh;

    mouse_pub = nh.advertise<ros_input_output::gui_event>("/mouse/read", 1);
    ros::Subscriber mouse_sub = nh.subscribe("/mouse/write", 1, mouseCallback);


    XRecordContext xrd;
    XRecordRange *range;
    XRecordClientSpec client;


    d0 = XOpenDisplay(NULL);
    d1 = XOpenDisplay(NULL);

    XSynchronize(d0, True);
    if (d0 == NULL || d1 == NULL) {
        fprintf(stderr, "Cannot connect to X server");
        exit (-1);
    }

    client=XRecordAllClients;

    range=XRecordAllocRange();
    memset(range, 0, sizeof(XRecordRange));
    range->device_events.first=ButtonPress;
    range->device_events.last=MotionNotify;

    xrd = XRecordCreateContext(d0, 0, &client, 1, &range, 1);

    if (! xrd) {
        fprintf(stderr, "Error in creating context");
        exit (-1);
    }

    XRecordEnableContextAsync(d1, xrd, event_callback, NULL);

    while (ros::ok())
    {
        XRecordProcessReplies (d1);
        ros::spinOnce();
    }



    XRecordDisableContext (d0, xrd);
    XRecordFreeContext (d0, xrd);



    XCloseDisplay(d0);
    XCloseDisplay(d1);

    ros::shutdown();

    exit(0);
    return 0;

}

