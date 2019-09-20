#include "TrajectoryDownloader/JointTrajectoryDownloader.h"

int main(int argc, char** argv)
{
	// initialize node
	ros::init(argc, argv, "indy7d_motion_downloader");
	
	// launch the default JointTrajectoryDownloader connection/handlers
	JointTrajectoryDownloader jmotionInterface(7);
	jmotionInterface.init();
	jmotionInterface.run();

  return 0;
}