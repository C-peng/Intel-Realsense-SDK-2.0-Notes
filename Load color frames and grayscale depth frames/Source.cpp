#include "opencv2\opencv.hpp"
#include "librealsense2\rs.hpp"



using namespace std;
using namespace cv;


void rs2AutoExposeStablizer(rs2::frameset &frames, rs2::pipeline &pipe) {
	for (int i = 0; i < 30; i++) frames = pipe.wait_for_frames();
}


int main() try 
{
	// For loading frames
	rs2::pipeline feed;
	
	// For mapping depth result to black and white
	rs2::colorizer colorizer;
	colorizer.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 1.f);
	colorizer.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);

	// For configuring rgb image
	rs2::config cfg, cfg2;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	
	// Start streaming, and feed in our configuration of RGB & depth frames
	feed.start(cfg);

	// Declare variable for holding frames
	rs2::frameset data;

	// Capture 30 frames prior to showing images on screen for auto exposure to warm up
	rs2AutoExposeStablizer(data, feed);


	while (waitKey(1) < 0) {
		data = feed.wait_for_frames();

		//rs2::frame depth = colorizer(data.get_depth_frame());
		rs2::frame color = data.get_color_frame();
		rs2::frame depth = colorizer(data.get_depth_frame());


		int w = color.as<rs2::video_frame>().get_width();
		int h = color.as<rs2::video_frame>().get_height();


		//Mat depth_frame(Size(w, h), CV_8UC3, (void*)depth.get_data());
		Mat color_frame(Size(w, h), CV_8UC3, (void*)color.get_data());
		Mat depth_frame(Size(w, h), CV_8UC3, (void*)depth.get_data());

		imshow("color", color_frame);
		imshow("depth", depth_frame);
	}
	
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
