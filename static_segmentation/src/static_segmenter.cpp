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

namespace static_segmenter{

static_segment::static_segment(ros::NodeHandle &nh) :
		nh_(nh),nh_priv_("~"){

	nh_priv_.param<std::string>("tabletop_service",tabletop_service_,std::string("/tabletop_segmentation"));

	nh_priv_.param<std::string>("graph_service",graph_service_,std::string("/graph_segment_srv"));

	nh_priv_.param<std::string>("rgb_input",rgb_topic_,std::string("/Honeybee/left/image_rect_color"));

}

geometry_msgs::Polygon static_segment::computeCGraph(){

	//convert
}

bool static_segment::serviceCallback(StaticSegment::Request &request, StaticSegment::Response &response){

	// recieving input rgb image
	sensor_msgs::Image::ConstPtr recent_color =
			ros::topic::waitForMessage<sensor_msgs::Image>(rgb_topic_, nh_, ros::Duration(5.0));

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
		response.c_graph.polygon = computeCGraph();
		response.c_graph.header.stamp = recent_color.header.stamp;
		response.c_graph.header.frame_id = recent_color.header.frame_id;
	}


}


}
