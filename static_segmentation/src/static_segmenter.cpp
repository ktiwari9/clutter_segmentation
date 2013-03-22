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
#include "static_segmentation/StaticSegment.h"
#include "static_segmentation/static_segmenter.hpp"
#include "static_segmentation/StaticSeg.h"
#include <graph_module/graph_module.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>

bool DEBUG = false;

namespace static_segmentation {

template<typename P>
struct CapacityGreater: public std::binary_function<std::vector<P>,
std::vector<P>, bool> {
  bool operator()(const std::vector<P> &a, const std::vector<P> &b) const {
    return a.size() > b.size();
  }
};

static_segment::static_segment(ros::NodeHandle &nh) :
		            nh_(nh),nh_priv_("~"){

  nh_priv_.param<std::string>("tabletop_service",tabletop_service_,std::string("/tabletop_segmentation"));

  nh_priv_.param<std::string>("graph_service",graph_service_,std::string("/graph_segment_srv"));

  nh_priv_.param<std::string>("rgb_input",rgb_topic_,std::string("/Honeybee/left/image_rect_color"));

  nh_priv_.param<std::string>("camera_info_topic",camera_topic_,std::string("/Honeybee/left/camera_info"));

  nh_priv_.param<double>("similarity_thresh",threshold_,100.0);

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

geometry_msgs::Point32 static_segment::createPoint32( double x, double y, double z ){

  geometry_msgs::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}


std::vector<StaticSeg> static_segment::computeCGraph(sensor_msgs::ImagePtr &return_image,
		std::vector<sensor_msgs::Image>& return_masks){

  //convert clusters to honeybee frame
  ROS_INFO("Transforming clusters to bumblebee frame");

  std::vector<sensor_msgs::PointCloud2> clusters;
  for (int i = 0; i < (int) tabletop_srv_.response.clusters.size(); i++) {

    // transforming every cluster in the service
    sensor_msgs::PointCloud2 transform_cloud;

    /*	bool can_transform = false;
		  while (!can_transform)
		  {
		    ROS_WARN("Waiting for transform between %s and %s\n", tabletop_srv_.response.clusters[0].header.frame_id.c_str(),
		    		graphsegment_srv_.response.segment.header.frame_id.c_str());
		    try
		    {
		      can_transform = listener_.waitForTransform(tabletop_srv_.response.clusters[0].header.frame_id,
		    		  graphsegment_srv_.response.segment.header.frame_id,graphsegment_srv_.response.segment.header.stamp, ros::Duration(1.0));
		      ros::spinOnce();
		    }
		    catch (tf::TransformException& ex)
		    {
		      ROS_ERROR("%s", ex.what());
		    }
		  }
     */
    //	ROS_VERIFY(pcl_ros::transformPointCloud(graphsegment_srv_.response.segment.header.frame_id,
    //		tabletop_srv_.response.clusters[i], transform_cloud,
    //		listener_));

    //-------------->// TODO: CORRECT THIS NAIVE FIX LATER<------------------------//
    try {
      tabletop_srv_.response.clusters[i].header.stamp = ros::Time(0);// <---- Change this later
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
  // to fill cluster masks
  cv::Mat element = cv::getStructuringElement(2, cv::Size(5,5));

  /// Apply the dilation operation
  cv::dilate(mask_all.clone(), bw_new, element);

  // Now erode image to get distinct clusters

  std::vector<std::vector<cv::Point> > contours_unordered;
  cv::findContours(bw_new.clone(), contours_unordered, CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);

  // Now sort the contours and throw out the ones less than 10% of the largest contour
  CapacityGreater<cv::Point> compare_object;
  std::sort(contours_unordered.begin(), contours_unordered.end(), compare_object);

  std::vector<std::vector<cv::Point> > contours_all;
  std::vector<cv::Point> row;

  // TODO : This is a bad hack but add this value as a param?
  int min_size = round(0.10*contours_unordered[0].size());

  // Preprocessing step for noise and outlier rejection
  for(int i = 0; i < (int)contours_unordered.size(); i++){

    if((int)contours_unordered[i].size() > min_size)
    {
      for(int j = 0; j < (int)contours_unordered[i].size(); j++)
        row.push_back(contours_unordered[i][j]);

      contours_all.push_back(row);
      row.clear();
    }
  }

  cv::Mat contour_mask = cv::Mat::zeros(mask_all.size(), CV_8UC1);
  cv::drawContours(contour_mask, contours_all, -1, cv::Scalar::all(255), CV_FILLED,8);

  ROS_INFO("Numbers of Contours found %d",(int)contours_all.size());

  // Copy only part of image that belongs to tabletop clusters and create a grayscale version of it
  cv::Mat segmented_image = returnCVImage(graphsegment_srv_.response.segment);
  cv::Mat segmented_gray;
  cv::cvtColor(segmented_image, segmented_gray, CV_BGR2GRAY);

  // An image to store the final set of all cluster graphs
  cv::Mat final_clusters = cv::Mat::zeros(segmented_gray.size(), CV_8UC1);
  // Now loop through the min max values and construct the separation graph
  cv::Mat center_graph = cv::Mat::zeros(segmented_gray.size(), CV_8UC1);

  // Creating a vector to store all the graphs in the image
  graph_list_.clear();
  //graph_list_.resize((int)contours_all.size());
  graph_centroid_.clear();

  std::vector<cv::Mat> gray_mats;
  // Now loop through the number of contours and construct separate graphs for each
  for(int i = 0; i < (int)contours_all.size(); i++){

    // now create a mask for every contour
    cv::Mat sub_contour_mask = cv::Mat::zeros(mask_all.size(), CV_8UC1);
    cv::Mat gray_graph = cv::Mat::zeros(segmented_gray.size(), CV_8UC1);

    // TODO: This is a stupid hack: there has to be a better way to do this
    std::vector<std::vector<cv::Point> > temp_contours;
    temp_contours.push_back(contours_all[i]);
    cv::drawContours(sub_contour_mask, temp_contours, -1, cv::Scalar::all(255), CV_FILLED,8);

    // Now compute the centroid of this cluster mask
    // To compute image centroid of the pixel mask
    cv::Moments m_c = cv::moments(sub_contour_mask, false);
    cv::Point2f sub_mask_center(m_c.m10/m_c.m00, m_c.m01/m_c.m00);

    // min and max value by converting into gray scale image
    double min_val, max_val;
    cv::minMaxLoc(segmented_gray,&min_val,&max_val,NULL,NULL,sub_contour_mask);

    //constructing return image
    ROS_INFO("Constructing the graph of all the clusters in range %f to %f", min_val,max_val);

    segmented_gray.copyTo(gray_graph,sub_contour_mask);

    // clearing cluster index list
    node_list_.clear();

    for(int j=(int)min_val;j<=(int)max_val;j++){

      cv::Mat cluster_mask = gray_graph == (double)j;
      cv::threshold(cluster_mask, cluster_mask, 20, 255, CV_THRESH_BINARY);

      if(cv::sum(cluster_mask).val[0] > 100000.0){ //TODO:change this heuristic later

        ROS_DEBUG("Image sum value %f",(double)cv::sum(cluster_mask).val[0]);

        // To compute image centroid of the pixel mask
        cv::Moments m = cv::moments(cluster_mask, false);
        cv::Point2f mask_center(m.m10/m.m00, m.m01/m.m00);

        //Populating the return polygon
        graph_node local_node;
        local_node.index_ = j;
        local_node.hist_ = computePatchFeature(input_,cluster_mask);
        local_node.x_ = mask_center.x; local_node.y_ = mask_center.y;

        node_list_.push_back(local_node);

        // Annotating the image
        cv::add(cluster_mask, final_clusters, final_clusters);
        cv::circle(center_graph, mask_center, 4, cv::Scalar(128,0,0), -1,8,0);
        cv::circle(cluster_mask, mask_center, 4, cv::Scalar(128,0,0), -1,8,0);

      }
    }

    // Now push the graph into the vector of graphs
    if(DEBUG){
      std::stringstream name;
      name<<"/tmp/cluster_mask_"<<i<<".png";
      cv::imwrite(name.str().c_str(),gray_graph);
    }

    graph_list_.push_back(node_list_);
    graph_centroid_.push_back(sub_mask_center);
    gray_mats.push_back(gray_graph);
  }

  // Data logging steps
  //cv::Mat return_cvMat = cv::Mat::zeros(segmented_image.size(), input_.type());

  // TODO: Check here if segmentation or transport fails!!
  cv::Mat return_cvMat = cv::Mat::zeros(segmented_gray.size(), segmented_gray.type());
  segmented_gray.copyTo(return_cvMat,final_clusters);
  //input_.copyTo(return_cvMat,final_clusters);

  //gray_graph = cv::Mat::zeros(segmented_image.size(), CV_8UC1);
  //segmented_gray.copyTo(gray_graph,final_clusters);
  cv::imwrite("/tmp/input_image.png",input_);
  cv::imwrite("/tmp/seg_image.png",segmented_image);
  cv::imwrite("/tmp/contour_mask.png",contour_mask);
  cv::imwrite("/tmp/cluster_image.png",mask_all);
  cv::imwrite("/tmp/graph_image.png",center_graph);

  // convert return image into sensor_msgs/ImagePtr
  cv_bridge::CvImage out_image;

  //Get header and encoding from your input image
  out_image.header   = graphsegment_srv_.response.segment.header;
  out_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
		  //graphsegment_srv_.response.segment.encoding;
  out_image.image    = return_cvMat;
  return_image = out_image.toImageMsg();

  //cv::imwrite("/tmp/graph_image.png",return_cvMat);
  cv::imwrite("/tmp/source_graph.png",center_graph);

  std::vector<StaticSeg> return_vector; // TODO: Place holder to compile

  ROS_INFO_STREAM("Sanity Check:\n number of graphs registered :"<<graph_list_.size()
                  <<" Number of centroids registered: "<<graph_centroid_.size());
  // Convert to output message type
  for(int i = 0; i < (int)graph_list_.size();i++)
  {
    StaticSeg single_seg;
    single_seg.graph = buildEGraph(graph_list_[i],gray_mats[i]);
    single_seg.centroid.x = graph_centroid_[i].x; single_seg.centroid.y = graph_centroid_[i].y;
    single_seg.centroid.z = 1;
    cv_bridge::CvImage conv_image;
    conv_image.header = graphsegment_srv_.response.segment.header;
    conv_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;

    conv_image.image = gray_mats[i];
    return_masks.push_back(*conv_image.toImageMsg());
    return_vector.push_back(single_seg);
  }

  ROS_INFO("Return vector built");

  return return_vector;

}

cv::MatND static_segment::computePatchFeature(cv::Mat input, cv::Mat mask){

  cv::MatND hist;
  cv::Mat hsv;

  ROS_DEBUG("Compute Patch Feature");

  cv::cvtColor(input,hsv,CV_BGR2HSV);

  // let's quantize the hue to 30 levels
  // and the saturation to 32 levels
  int hbins = 30, sbins = 32;
  int histSize[] = {hbins, sbins};
  // hue varies from 0 to 179, see cvtColor
  float hranges[] = { 0, 180 };
  // saturation varies from 0 (black-gray-white) to
  // 255 (pure spectrum color)
  float sranges[] = { 0, 256 };
  const float* ranges[] = { hranges, sranges };
  // we compute the histogram from the 0-th and 1-st channels
  int channels[] = {0, 1};

  cv::calcHist( &hsv, 1, channels, mask, // do not use mask
            hist, 2, histSize, ranges,
            true, // the histogram is uniform
            false );

  return hist;
}

void static_segment::addEdge(local_graph_it it_1, local_graph_it it_2, graph::ros_graph& graph){

  graph::Vertex_ros v1,v2;
  v1.index_ = it_1->index_;
  v1.x_ = it_1->x_; v1.y_ = it_1->y_;
  v2.index_ = it_2->index_;
  v2.x_ = it_2->x_;v2.y_ = it_2->y_;

  if(!graph.findEdge(v1,v2)){
    ROS_DEBUG("Adding before Edge between %d and %d",v1.index_,v2.index_);
    graph.addEdge(v1,v2,1); // TODO: Think about adding distance between nodes as weight
  }

}

graph_module::EGraph static_segment::buildEGraph(std::vector<graph_node> node_list, cv::Mat segment){

  // construct graph
  ROS_INFO("Constructing E graph Message");
  graph::ros_graph e_graph(node_list.size());

  if(e_graph.number_of_vertices_ > 1)
  {
    std::vector<graph_node>::iterator node_it_end;

    // now loop through image and construct graph from connected components
    int rows = segment.rows, cols = segment.cols;

    for(int i = 0 ; i < rows ; i++)
      for(int j = 0 ; j < cols; j++){

        if((int)segment.at<uchar>(i,j) > 0){

          node_it_end = std::find_if(node_list.begin(),node_list.end(),find_node((int)segment.at<uchar>(i,j)));
          //if node in list
          if(node_it_end!=node_list.end()){

            //check if this is the first pixel in the image
            if(i == 0 && j == 0)
              continue;

            //checking the first row and west value in the image
            if(j != 0){
              if((int)segment.at<uchar>(i,j) != (int)segment.at<uchar>(i,j-1)){

                local_graph_it node_it_ = std::find_if(node_list.begin(),node_list.end(),find_node((int)segment.at<uchar>(i,j-1)));
                if(node_it_!=node_list.end())
                  addEdge(node_it_,node_it_end,e_graph);
              }
            }

            //checking all the other rows (North Value)
            if(i != 0){
              if((int)segment.at<uchar>(i,j) != (int)segment.at<uchar>(i-1,j))
              {
                local_graph_it node_it_ = std::find_if(node_list.begin(),node_list.end(),find_node((int)segment.at<uchar>(i-1,j)));
                if(node_it_!=node_list.end())
                  addEdge(node_it_,node_it_end,e_graph);
              }
            }
          }
        }
      }
  }
  else
  {
    // To account for singular nodes, ie point graphs that only have one node
    local_graph_it node_it_ = node_list.begin();
    addEdge(node_it_,node_it_,e_graph);

  }


  for(int i=0;i< (int)node_list.size();i++)
    ROS_DEBUG("Printing E-Graph Values %d",node_list[i].index_);

  return e_graph.convertToEGraphMsg();

}



bool static_segment::serviceCallback(StaticSegment::Request &request, StaticSegment::Response &response){

  // receiving input rgb image
  sensor_msgs::Image::ConstPtr recent_color =
      ros::topic::waitForMessage<sensor_msgs::Image>(rgb_topic_, nh_, ros::Duration(5.0));

  sensor_msgs::CameraInfo::ConstPtr cam_info =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_topic_, nh_, ros::Duration(5.0));

  cam_info_ = *cam_info;

  //convert input rgb image into cv::Mat
  graphsegment_srv_.request.rgb = *recent_color;
  input_ = returnCVImage(*recent_color);

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
    response.graph_queue = computeCGraph(graph_image,response.cluster_masks);
    response.graph_image = *graph_image;
    response.graph_image.header.stamp = recent_color->header.stamp;
    response.graph_image.header.frame_id = recent_color->header.frame_id;
    response.result = response.SUCCESS;
    return true;
  }
}

}

int main(int argc, char **argv){

  ros::init(argc, argv, "static_segment");
  ros::NodeHandle nh;

  static_segmentation::static_segment ss(nh);

  ros::spin();
  return 0;

}
