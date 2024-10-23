#include <ros/ros.h>
#include <tyrion_dwa.h>
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>




/* 
class OctomapServerNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    NODELET_DEBUG("Initializing octomap server nodelet ...");
    ros::NodeHandle& nh = this->getNodeHandle();
    ros::NodeHandle& private_nh = this->getPrivateNodeHandle();
    server_.reset(new OctomapServer(private_nh, nh));

    std::string mapFilename("");
    if (private_nh.getParam("map_file", mapFilename)) {
      if (!server_->openFile(mapFilename)){
        NODELET_WARN("Could not open file %s", mapFilename.c_str());
      }
    }
  }
private:
  boost::shared_ptr<OctomapServer> server_;
};


PLUGINLIB_EXPORT_CLASS(octomap_server::OctomapServerNodelet, nodelet::Nodelet)
 */