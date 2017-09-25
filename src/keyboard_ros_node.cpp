#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
#include "ros_input_output/gui_event.h"

#include <sstream>


//#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xproto.h>
#include <X11/extensions/record.h>
#include <X11/keysym.h>
#include <xosd.h>


#define KEY_PRESS true
#define KEY_RELEASE false

/* for this struct, refer to libxnee */
typedef union {
    unsigned char type ;
    xEvent event ;
    xResourceReq req ;
    xGenericReply reply ;
    xError error ;
    xConnSetupPrefix setup;
} XRecordDatum;





ros::Publisher key_pub_;
ros::Subscriber key_sub_;

Display *d0, *d1;




void event_callback(XPointer priv, XRecordInterceptData *hook)
{
    /* FIXME: we need use XQueryPointer to get the first location */
//	std_msgs::String msg;
    ros_input_output::gui_event gui_event;


    static int cur_x = 0;
    static int cur_y = 0;
    if (hook->category != XRecordFromServer)
    {
        XRecordFreeData (hook);
        return;
    }
    XRecordDatum *data = (XRecordDatum*) hook->data;
    int event_type = data->type;
    BYTE btncode, keycode;
    btncode = keycode = data->event.u.u.detail;
    switch (event_type) {
    case KeyPress:

        /* if escape is pressed, stop the loop and clean up, then exit */
        //if (keycode == 9) stop = 1;
        /* Note: you should not use data_disp to do normal X operations !!!*/

        // if (key_pub_.getNumSubscribers() > 0)
        //{
        //msg.data = XKeysymToString(XKeycodeToKeysym(d0, keycode, 0));
        //key_pub_.publish(msg);
        //ros::spinOnce();

        gui_event.header.stamp = ros::Time::now();
        gui_event.type = ros_input_output::gui_event::KEYBOARD_EVENT_PRESS;

        gui_event.mouse_x = cur_x;
        gui_event.mouse_y = cur_y;
        if (btncode == 1)
            gui_event.mouse_left = 1;
        else if (btncode == 3)
            gui_event.mouse_right = 1;

        gui_event.btncode = btncode;

        key_pub_.publish(gui_event);


        //}

        break;
    case KeyRelease:
        //printf ("KeyRelease: \t%s\n", XKeysymToString(XKeycodeToKeysym(d0, keycode, 0)));

        gui_event.header.stamp = ros::Time::now();
        gui_event.type = ros_input_output::gui_event::KEYBOARD_EVENT_RELEASE;

        gui_event.mouse_x = cur_x;
        gui_event.mouse_y = cur_y;
        if (btncode == 1)
            gui_event.mouse_left = 0;
        else if (btncode == 3)
            gui_event.mouse_right = 0;

        gui_event.btncode = btncode;

        key_pub_.publish(gui_event);


        break;
    default:
        break;
    }
//	printf (", time=%d\n", time);

    XRecordFreeData (hook);
}


// Function to create a keyboard event
XKeyEvent createKeyEvent(Display *display, Window &win,
                         Window &winRoot, bool press,
                         int keycode, int modifiers)
{
    XKeyEvent event;

    event.display     = display;
    event.window      = win;
    event.root        = winRoot;
    event.subwindow   = None;
    event.time        = CurrentTime;
    event.x           = 1;
    event.y           = 1;
    event.x_root      = 1;
    event.y_root      = 1;
    event.same_screen = True;
    event.keycode     = XKeysymToKeycode(display, keycode);
    event.state       = modifiers;

    if(press)
        event.type = KeyPress;
    else
        event.type = KeyRelease;

    return event;
}


void keyboardCallback(const ros_input_output::gui_event::ConstPtr& msg)
{
    if (msg->type != ros_input_output::gui_event::KEYBOARD_EVENT_PRESS && msg->type != ros_input_output::gui_event::KEYBOARD_EVENT_RELEASE) return;



    Display *display = d0;
    Window winRoot = XDefaultRootWindow(display);
    Window winFocus;
    int    revert;
    XGetInputFocus(display, &winFocus, &revert);

    XKeyEvent event;

    switch (msg->type)
    {
    case ros_input_output::gui_event::KEYBOARD_EVENT_PRESS:

        event = createKeyEvent(display, winFocus, winRoot, KEY_PRESS, msg->btncode, 0);
        XSendEvent(event.display, event.window, True, KeyPressMask, (XEvent *)&event);
        event = createKeyEvent(display, winFocus, winRoot, KEY_RELEASE, msg->btncode, 0);
        XSendEvent(event.display, event.window, True, KeyPressMask, (XEvent *)&event);

        break;
    case ros_input_output::gui_event::KEYBOARD_EVENT_RELEASE:
        break;
    default:
        break;
    }


    //std_msgs::String str_msg = *msg;
//	const char *str = str_msg.data.c_str();



    /*	for (unsigned int i = 0; i < str_msg.data.length(); i++)
    	{
    	   XKeyEvent event = createKeyEvent(display, winFocus, winRoot, true, str[i], 0);
    	   XSendEvent(event.display, event.window, True, KeyPressMask, (XEvent *)&event);
    	   event = createKeyEvent(display, winFocus, winRoot, false, str[i], 0);
    	   XSendEvent(event.display, event.window, True, KeyPressMask, (XEvent *)&event);
    	}*/
}



int
main(int argc, char **argv)
{

    ros::init(argc, argv, "keyboard_node"); //test
    ros::NodeHandle nh;
    key_pub_ = nh.advertise<ros_input_output::gui_event>("/keyboard/read", 1);
    key_sub_ = nh.subscribe("/keyboard/write", 1, keyboardCallback);


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
    range->device_events.first=KeyPress;
    range->device_events.last=KeyRelease;

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

    ros::shutdown();

    XCloseDisplay(d0);
    XCloseDisplay(d1);
    exit(0);
    return 0;
}

