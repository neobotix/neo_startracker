#include "startracker.h"



int main(int argc, char *argv[])
{	
    ros::init(argc, argv, "neo_startracker_node");
    StarTracker tracker;

    //ros  rate fehlt!

    int ret = 0;
    ret = tracker.init();

	
    while(ros::ok() && ret == 0)
	{
        tracker.run();
		
		ros::spinOnce();
	}

    tracker.shutdown();
	return 0;
}

