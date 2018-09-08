#include "opencv2\opencv.hpp"
#include "librealsense2\rs.hpp"

using namespace std;
using namespace cv;

string AUTHOR = "Wei Chih Chern, on ";
string DATE   = "09/08/2018\n";
string SDKver = "Intel RealSense v2.15.0 & OpenCV 3.4\n";
string knownISSUE = "*Current RealSense version align() function isn't well optimized, which causing low fps. Big thanks to the SDK developer and wish the problem can be solved soon!\n";
string FYI = "*This App is an OpenCV version of the example on RealSense Github which can be found in its Github /example/align.\n";

const int MAX_DISTANCE = 10; // 10 Meters


float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
	//Given a vector of streams, we try to find a depth stream and another stream to align depth with.
	//We prioritize color streams to make the view look better.
	//If color is not available, we take another stream that (other than depth)
	rs2_stream align_to = RS2_STREAM_ANY;
	bool depth_stream_found = false;
	bool color_stream_found = false;
	for (rs2::stream_profile sp : streams)
	{
		rs2_stream profile_stream = sp.stream_type();
		if (profile_stream != RS2_STREAM_DEPTH)
		{
			if (!color_stream_found)     	//Prefer color
				align_to = profile_stream;

			if (profile_stream == RS2_STREAM_COLOR)
			{
				color_stream_found = true;
			}
		}
		else
		{
			depth_stream_found = true;
		}
	}

	if (!depth_stream_found)
		throw std::runtime_error("No Depth stream available");

	if (align_to == RS2_STREAM_ANY)
		throw std::runtime_error("No stream found to align with Depth");

	return align_to;
}

void distThreshold_BW(const Mat &depth_frame, Mat &dst, float depth_scale, float distThres)
{
	if (depth_frame.type() != CV_16UC1) { cout << "Depth frame captured by Realsense SDK 2.0 should be 16 bits depth.\n\n"; return; }

	int cols = depth_frame.cols;
	int rows = depth_frame.rows;

	dst = Mat::zeros(Size(cols, rows), CV_8U);

#pragma omp parallel for 
	for (int i = 0; i < rows; i++)
	{
		const uint16_t *depth_ptr = depth_frame.ptr<uint16_t>(i);
		uchar *dst_ptr = dst.ptr<uchar>(i);
		for (int j = 0; j < cols; j++)
		{
			auto dist = depth_scale * depth_ptr[j];
			dst_ptr[j] = dist < distThres ? UINT8_MAX : 0;
		}
	}
}

void distThreshold_Color(const Mat &depth_frame, const Mat &color, Mat &dst, float depth_scale, float distThres)
{
	if (depth_frame.type() != CV_16UC1) { cout << "Depth frame captured by Realsense SDK 2.0 should be 16 bits depth.\n\n"; return; }

	int cols = depth_frame.cols;
	int rows = depth_frame.rows;

	dst = Mat::zeros(Size(cols, rows), CV_8UC3);

#pragma omp parallel for 
	for (int i = 0; i < rows; i++)
	{
		const uint16_t *depth_ptr = depth_frame.ptr<uint16_t>(i); //1 Channel
		const uchar    *color_ptr = color.ptr<uchar>(i);          //3 Channels
		      uchar      *dst_ptr = dst.ptr<uchar>(i);            //3 Channels

		int x = 0;

		for (int j = 0; j < cols; j++, x=j*3)
		{
			auto dist = depth_scale * depth_ptr[j];
			if (dist < distThres)
			{
				dst_ptr[x] = color_ptr[x];
				dst_ptr[x+1] = color_ptr[x+1];
				dst_ptr[x+2] = color_ptr[x+2];
			}
		}
	}
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
	for (auto&& sp : prev)
	{
		//If previous profile is in current (maybe just added another)
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) //If it previous stream wasn't found in current
		{
			return true;
		}
	}
	return false;
}

void autoExposureWarmUp(rs2::pipeline &feed, int nFrames = 30)
{
	for (int i = 0; i < nFrames; i++) feed.wait_for_frames();
}

void userManual()
{
	cout << "Press key 'A' to increase distance by 0.2 meter.\n";
	cout << "Press key 'S' to decrease distance by 0.2 meter.\n";
	cout << "Press key 'Z' to increase distance by 0.1 meter.\n";
	cout << "Press key 'Q' to exit the program.\n\n";
}

void appInfo()
{
	cout << "-------------------------------------------------------------------------------------------------------------------\n";
	cout << AUTHOR;
	cout << DATE;
	cout << SDKver;
	cout << knownISSUE;
	cout << FYI;
	cout << "-------------------------------------------------------------------------------------------------------------------\n\n\n\n";
}

int main() try
{
	rs2::pipeline  feed;                            // For loading frames

	rs2::config	cfg;                              	// Capture 30 frames prior to showing images on screen for auto exposure to warm up
	int w = 640, h = 480, fps = 30;
	cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
	cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);
	rs2::pipeline_profile profile = feed.start(cfg);	// Start streaming, and feed in our configuration of RGB & depth frames

	float depthScale = get_depth_scale(profile.get_device());    	// Get depth scale to turn depth value into distance (meters)
	rs2_stream align_to = find_stream_to_align(profile.get_streams());  // Find a stream for alignment, color stream is preferred
	rs2::align align(align_to);

	rs2::frameset data;                             	// Declare variable for holding frames    
	autoExposureWarmUp(feed);

	double distance_threshold_value = 2.0;


	appInfo();
	userManual();

	while (true)
	{

		data = feed.wait_for_frames();

		data = align.process(data); 
		rs2::video_frame color = data.get_color_frame();
		rs2::depth_frame depth = data.get_depth_frame(); 

		Mat colorFrame(Size(w, h), CV_8UC3, (void*)color.get_data());
		Mat depthFrame(Size(w, h), CV_16UC1, (void*)depth.get_data());

		cout << "Current distance threshold value is: " << distance_threshold_value << "\r";

		Mat result; 
		distThreshold_Color(depthFrame, colorFrame, result, depthScale, distance_threshold_value);
		imshow("Distance threshold result", result);

		char c = waitKey(1);

		if      (c == 'a' || c == 'A') distance_threshold_value += 0.2;
		else if (c == 's' || c == 'S') distance_threshold_value -= 0.2;
		else if (c == 'z' || c == 'Z') distance_threshold_value += 0.1;
		else if (c == 'q' || c == 'Q') return 0;

		if (distance_threshold_value < 0)            distance_threshold_value = 0.0;
		if (distance_threshold_value > MAX_DISTANCE) distance_threshold_value = 10.0;
		
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n	" << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
