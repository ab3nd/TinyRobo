#include <neighbor_sense/neighbor_sense.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "");
	ros::NodeHandle node("~");

	NeighborSense ns = NeighborSense(node);

	ros::spin();
}
