/*
 * Copyright (c) 2012, University of Southern California, CLMC Lab
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \author Bharath Sankaran
 *
 * @b Static Segmentation merging class
 */
#include "static_segmentation/static_segmenter.hpp"

namespace static_segmentation{

static_segment::static_segment(ros::NodeHandle &nh) :
		nh_(nh),nh_priv_("~"){

	nh_priv_.param<std::string>("tabletop_service",tabletop_service_,std::string("/tabletop_segmentation"));

	nh_priv_.param<std::string>("graph_service",graph_service_,std::string("/graph_segment_srv"));

	nh_priv_.param<std::string>("rgb_input",rgb_topic_,std::string("/Honeybee/left/image_rect_color"));

	nh_priv_.param<std::string>("camera_info_topic",camera_topic_,std::string("/Honeybee/left/camera_info"));

	static_segment_srv_ = nh_.advertiseService(nh_.resolveName("static_segment_srv"),&static_segment::serviceCallback, this);



}

static_segment::~static_segment(){ }

void static_segment::getMasksFromClusters(const std::vector<sensor_msgs::PointCloud2> &clusters,
		const sensor_msgs::CameraInfo &cam_info, std::vector<sensor_msgs::Image> &masks) {

	masks.resize(clusters.size());

	Eigen::Matrix4f P;
	int rows = 3, cols = 4;
	for (int r = 0; r < rows; ++r)
		for (int c = 0; c < cols; ++c)
			P(r, c) = cam_info.P[r * cols + c];

	P(3, 0) = 0;
	P(3, 1) = 0;
	P(3, 2) = 0;
	P(3, 3) = 1;

	//std::cout << "Transformation Matrix " << std::endl << P << std::endl;

	for (size_t i = 0; i < clusters.size(); ++i) {

		sensor_msgs::PointCloud2 cloud_proj;

		sensor_msgs::Image mask;
		mask.height = cam_info.height;
		mask.width = cam_info.width;
		//mask.encoding = enc::TYPE_32FC1;
		mask.encoding = sensor_msgs::image_encodings::MONO8;
		mask.is_bigendian = false;
		mask.step = mask.width;
		size_t size = mask.step * mask.height;
		mask.data.resize(size);

		pcl_ros::transformPointCloud(P, clusters[i], cloud_proj);

		for (unsigned int j = 0; j < cloud_proj.width; j++) {

			float x, y, z;

			memcpy(&x,
					&cloud_proj.data[j * cloud_proj.point_step
							+ cloud_proj.fields[0].offset], sizeof(float));
			memcpy(&y,
					&cloud_proj.data[j * cloud_proj.point_step
							+ cloud_proj.fields[1].offset], sizeof(float));
			memcpy(&z,
					&cloud_proj.data[j * cloud_proj.point_step
							+ cloud_proj.fields[2].offset], sizeof(float));

			if (round(y / z) >= 0 && round(y / z) < mask.height
					&& round(x / z) >= 0 && round(x / z) < mask.width) {
				int i = round(y / z) * mask.step + round(x / z);
				mask.data[i] = 255;
			}
		}

		masks[i] = mask;
	}
}

cv::Mat static_segment::returnCVImage(const sensor_msgs::Image & img) {

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		exit(0);
	}

	return cv_ptr->image;
}


geometry_msgs::Polygon static_segment::computeCGraph(sensor_msgs::ImagePtr &return_image){

	//convert clusters to honeybee frame
	ROS_INFO("Transforming clusters to bumblebee frame");

	std::vector<sensor_msgs::PointCloud2> clusters;
	for (int i = 0; i < (int) tabletop_srv_.response.clusters.size(); i++) {

		// transforming every cluster in the service
		sensor_msgs::PointCloud2 transform_cloud;

		try {
			//segmentation_srv.response.clusters[i].header.stamp = ros::Time(0);// <---- Change this later
			pcl_ros::transformPointCloud(graphsegment_srv_.response.segment.header.frame_id,
					tabletop_srv_.response.clusters[i], transform_cloud,
					listener_);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("Failed to transform cloud from frame %s into frame %s", tabletop_srv_.response.clusters[0].header.frame_id.c_str(),
					graphsegment_srv_.response.segment.header.frame_id.c_str());
		}

		clusters.push_back(transform_cloud);
	}

	ROS_INFO("Transformed clusters");

	// Now getting transformed masks from transformed clusters
	std::vector<sensor_msgs::Image> bbl_masks;
	getMasksFromClusters(clusters, cam_info_, bbl_masks);

	// Now combine masks and add to static segmenter
	cv::Mat init = returnCVImage(bbl_masks[0]);
	cv::Mat mask_all = cv::Mat::zeros(init.size(), CV_8UC1);

	for (int i = 0; i < (int) bbl_masks.size(); i++) {

		cv::Mat mask = returnCVImage(bbl_masks[i]);
		cv::add(mask, mask_all, mask_all);
	}

	// closing all contours via dialation
	cv::Mat bw_new;
	cv::Mat element = cv::getStructuringElement(2, cv::Size(5,5));

	/// Apply the dilation operation
	cv::dilate(mask_all.clone(), bw_new, element);

	std::vector<std::vector<cv::Point> > contours_all;
	cv::findContours(bw_new.clone(), contours_all, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	cv::Mat contour_mask = cv::Mat::zeros(mask_all.size(), CV_8UC1);
	cv::drawContours(contour_mask, contours_all, -1, cv::Scalar::all(255), CV_FILLED,8);

	// Copy only part of image that belongs to tabletop clusters
	cv::Mat segmented_mask = cv::Mat::zeros(mask_all.size(), mask_all.type());

	cv::Mat segmented_image = returnCVImage(graphsegment_srv_.response.segment);
	//segmented_image.copyTo(segmented_mask, contour_mask);

	cv::Mat segmented_gray;

	cv::cvtColor(segmented_image, segmented_gray, CV_BGR2GRAY);

	// min and max value
	double min_val, max_val;

	cv::minMaxLoc(segmented_gray,&min_val,&max_val,NULL,NULL,contour_mask);

	//constructing return image
	cv::Mat return_cvMat = cv::Mat::zeros(segmented_image.size(), segmented_image.type());
	segmented_image.copyTo(return_cvMat,contour_mask);

	// convert return image into sensor_msgs/ImagePtr
	cv_bridge::CvImage out_image;

	//Get header and encoding from your input image
	out_image.header   = graphsegment_srv_.response.segment.header;
	out_image.encoding = graphsegment_srv_.response.segment.encoding;
	out_image.image    = return_cvMat;

	return_image = out_image.toImageMsg();

	ROS_INFO("Constructing the graph of all the clusters");

	geometry_msgs::Polygon return_polygon;

	// Now loop through the min max values and construct the separation graph
	int count = 0;

	cv::Mat center_graph = cv::Mat::zeros(segmented_gray.size(), CV_8UC1);

	for(int i=(int)min_val;i<=(int)max_val;i++){

		cv::Mat cluster_mask = segmented_gray == (double)i;

		if(cv::sum(cluster_mask).val[0] > 0.0){

			// To compute image centroid of the pixel mask
			cv::Moments m = cv::moments(cluster_mask, false);
			cv::Point mask_center(m.m10/m.m00, m.m01/m.m00);

			//Populating the return polygon
			return_polygon.points[count].x = mask_center.x;
			return_polygon.points[count].y = mask_center.y;

			// Annotating the image
			cv::circle(cluster_mask, mask_center, 5, cv::Scalar(128,0,0), -1);
			cv::add(cluster_mask, center_graph, center_graph);
			count++;

		}
	}

	return return_polygon;
}

bool static_segment::serviceCallback(StaticSegment::Request &request, StaticSegment::Response &response){

	// recieving input rgb image
	sensor_msgs::Image::ConstPtr recent_color =
			ros::topic::waitForMessage<sensor_msgs::Image>(rgb_topic_, nh_, ros::Duration(5.0));

	sensor_msgs::CameraInfo::ConstPtr cam_info =
	    ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_topic_, nh_, ros::Duration(5.0));

	cam_info_ = *cam_info;

	//convert input rgb image into cv::Mat
	graphsegment_srv_.request.rgb = *recent_color;

	//calling graph based segmentation service
	if (!ros::service::call(graph_service_, graphsegment_srv_)) {
		ROS_ERROR("Call to graph segmentation failed");
		response.result = response.FAILURE;
		return false;
	}

	if (graphsegment_srv_.response.result == graphsegment_srv_.response.FAILED) {
		ROS_ERROR(" Graph based segmentation service returned error");
		response.result = response.FAILURE;
		return false;
	}

	// Calling tabletop segmentation service
	if (!ros::service::call(tabletop_service_, tabletop_srv_)) {
		ROS_ERROR("Call to tabletop segmentation service failed");
		response.result = response.FAILURE;
		return false;
	}
	if (tabletop_srv_.response.result != tabletop_srv_.response.SUCCESS
			&& tabletop_srv_.response.result != tabletop_srv_.response.SUCCESS_NO_RGB) {

		ROS_ERROR("Segmentation service returned error %d", tabletop_srv_.response.result);
		response.result = response.FAILURE;
		return false;
	}

	ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)tabletop_srv_.response.clusters.size());

	if (tabletop_srv_.response.clusters.empty()) {
		response.result = response.FAILURE;
		return false;
	}
	else{

		//If both services return success we project the graph clusters onto the euclidean clusters
		// and generate a connectivity graph
		sensor_msgs::ImagePtr graph_image;
		response.c_graph.polygon = computeCGraph(graph_image);
		response.graph_image = *graph_image;
		response.c_graph.header.stamp = recent_color->header.stamp;
		response.c_graph.header.frame_id = recent_color->header.frame_id;
		return true;
	}

}

}

int main(int argc, char **argv){

	ros::init(argc, argv, "graph_segment");
	ros::NodeHandle nh;

	static_segmentation::static_segment ss(nh);

	ros::spin();
	return 0;

}
