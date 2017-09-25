#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Byte.h"
#include "std_msgs/String.h"

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
#include <fcntl.h>


ros::Publisher keyboard_pub;

#define KEY_BUFF_SIZE 1024
static char key_buff[KEY_BUFF_SIZE];

char *TranslateKeyCode(XEvent *ev);

Window last_window;
Display *d;

void snoop_all_windows(Window root, unsigned long type)
{
    static int level = 0;
    Window parent, *children;
    unsigned int nchildren;
    unsigned int stat, i;
    level++;
    stat = XQueryTree(d, root, &root, &parent, &children, &nchildren);
    if (stat == FALSE) {
        fprintf(stderr, "Can't query window tree...\n");
        return;
    }
    if (nchildren == 0)
        return;

    XSelectInput(d, root, type);
    for (i = 0; i < nchildren; i++) {
        XSelectInput(d, children[i], type);
        snoop_all_windows(children[i], type);
    }
    XFree((char *) children);
}


void handleCreateEvent(XEvent *ev)
{
    XCreateWindowEvent *xcwe;
    xcwe=(XCreateWindowEvent *)ev;
    snoop_all_windows(xcwe->window, SubstructureNotifyMask|KeyPressMask);
    printf("-- New Window (0x%x) --\n", xcwe->window);
}


void handleKeyEvent(XEvent *ev)
{
    char *string=NULL;
    string = TranslateKeyCode(ev);
    if (string == NULL)
        return;

    /*if (*string == '\r')
    {
    	printf("\n");
    }
    else if (strlen(string) == 1)
    {
    	printf("%s", string);
    }
    else
    {
    	printf("<<%s>>", string);
    }*/

    std_msgs::String msg;
    msg.data = *string;

    keyboard_pub.publish(msg);

    fflush(stdout);
}





int main(int argc, char **argv)
{
    char *hostname;
    XEvent xev;
    hostname = ":0";

    ros::init(argc, argv, "keyboardgrab_ros_node");

    ros::NodeHandle nh;


    keyboard_pub = nh.advertise<std_msgs::String>("keyboard", 5);



    d = XOpenDisplay(hostname);

    if (d == NULL) {
        fprintf(stderr, "Blah, can't open display: %s\n", hostname);
        exit(10);
    }




    snoop_all_windows(DefaultRootWindow(d), SubstructureNotifyMask|KeyPressMask);


    while (ros::ok()) {
        XNextEvent(d, &xev);
        switch(xev.type) {
        case CreateNotify:
            handleCreateEvent(&xev);
            break;
        case KeyPress:
            handleKeyEvent(&xev);
            break;
        default:
            //ROS_INFO("unknown type = %x", xev.type);
            break;
        }

        ros::spinOnce();
    }

    return 0;

}



char *
TranslateKeyCode(XEvent * ev)
{
    int count;
    char *tmp;
    KeySym ks;
    Window current_window;

    if (ev) {
        XKeyEvent *xke = (XKeyEvent *) ev;
        current_window = xke->window;
        if (current_window != last_window) {
            printf("\n -- switched to window 0x%x--\n", current_window);
            last_window = current_window;
        }
        count = XLookupString(xke, key_buff, KEY_BUFF_SIZE, &ks, NULL);
        key_buff[count] = '\0';
        if (count == 0) {
            tmp = XKeysymToString(ks);
            if (tmp)
                strcpy(key_buff, tmp);
            else
                strcpy(key_buff, "");
        }
        return key_buff;
    }
    else
    {
        return NULL;
    }
}



