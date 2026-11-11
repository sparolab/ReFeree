#include "ltslam/Session.h"
#include "referee_slam/graph_io.hpp"

Session::Session()
{

}

Session::Session(int _idx, std::string _name, std::string _session_dir_path, bool _is_base_session)
    : index_(_idx), name_(_name), session_dir_path_(_session_dir_path), is_base_session_(_is_base_session)
{
    allocateMemory();
    
    loadSessionGraph();
    
    loadSessionDescriptors();
    loadSessionKeyframePointclouds();

    const float kICPFilterSize = 0.3; // TODO move to yaml 
    downSizeFilterICP.setLeafSize(kICPFilterSize, kICPFilterSize, kICPFilterSize);

} // ctor


void Session::allocateMemory()
{
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    originPoses6D.reset(new pcl::PointCloud<PointTypePose>());
}


void Session::initKeyPoses(void)
{
    for(auto & _node_info: nodes_)
    {
        PointTypePose thisPose6D;

        int node_idx = _node_info.first;
        Node node = _node_info.second; 
        gtsam::Pose3 pose = node.initial;

        thisPose6D.x = pose.translation().x();
        thisPose6D.y = pose.translation().x();
        thisPose6D.z = pose.translation().x();
        thisPose6D.intensity = node_idx; // TODO
        thisPose6D.roll  = pose.rotation().roll();
        thisPose6D.pitch = pose.rotation().pitch();
        thisPose6D.yaw   = pose.rotation().yaw();
        thisPose6D.time = 0.0; // TODO

        cloudKeyPoses6D->push_back(thisPose6D);    
    }

    PointTypePose thisPose6D;
    thisPose6D.x = 0.0;
    thisPose6D.y = 0.0;
    thisPose6D.z = 0.0;
    thisPose6D.intensity = 0.0;
    thisPose6D.roll = 0.0;
    thisPose6D.pitch = 0.0;
    thisPose6D.yaw = 0.0;
    thisPose6D.time = 0.0;
    originPoses6D->push_back( thisPose6D );
} // initKeyPoses


void Session::updateKeyPoses(const gtsam::ISAM2 * _isam, const gtsam::Pose3& _anchor_transform)
{
    using gtsam::Pose3;

    gtsam::Values isamCurrentEstimate = _isam->calculateEstimate();

    int numPoses = cloudKeyFrames.size();
    for (int node_idx_in_sess = 0; node_idx_in_sess < numPoses; ++node_idx_in_sess)
    {
        int node_idx_in_global = genGlobalNodeIdx(index_, node_idx_in_sess);
        // cout << "update the session " << index_ << "'s node: " << node_idx_in_sess << " (global idx: " << node_idx_in_global << ")" << endl;

        gtsam::Pose3 pose_self_coord = isamCurrentEstimate.at<Pose3>(node_idx_in_global);
        gtsam::Pose3 pose_central_coord = _anchor_transform * pose_self_coord;

        cloudKeyPoses6D->points[node_idx_in_sess].x = pose_central_coord.translation().x();
        cloudKeyPoses6D->points[node_idx_in_sess].y = pose_central_coord.translation().y();
        cloudKeyPoses6D->points[node_idx_in_sess].z = pose_central_coord.translation().z();
        cloudKeyPoses6D->points[node_idx_in_sess].roll  = pose_central_coord.rotation().roll();
        cloudKeyPoses6D->points[node_idx_in_sess].pitch = pose_central_coord.rotation().pitch();
        cloudKeyPoses6D->points[node_idx_in_sess].yaw   = pose_central_coord.rotation().yaw();
    }
} // updateKeyPoses


void Session::loopFindNearKeyframesCentralCoord(
    pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
    const int& key, const int& searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes += *transformPointCloud(cloudKeyFrames[keyNear], &cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    nearKeyframes->clear(); // redundant?
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCentralCoord


void Session::loopFindNearKeyframesLocalCoord(
    pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
    const int& key, const int& searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes += *transformPointCloud(cloudKeyFrames[keyNear], &originPoses6D->points[0]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    nearKeyframes->clear(); // redundant?
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesLocalCoord


void Session::loadSessionKeyframePointclouds()
{
    std::string pcd_dir = session_dir_path_ + "/Scans/";

    // parse names (sorted)
    std::map<int, std::string> pcd_names; // for auto-sort (keyframes must line up with the node indices)
    for(auto& _pcd : fs::directory_iterator(pcd_dir)) 
    {
        std::string pcd_name = _pcd.path().filename();

        std::stringstream pcd_name_stream {pcd_name};
        std::string pcd_idx_str; 
        getline(pcd_name_stream, pcd_idx_str, ',');
        int pcd_idx = std::stoi(pcd_idx_str);
        std::string pcd_name_filepath = _pcd.path();

        pcd_names.insert(std::make_pair(pcd_idx, pcd_name_filepath));
    }    

    // load PCDs
    int num_pcd_loaded = 0;
    for (auto const& _pcd_name: pcd_names)
    {
        // cout << " load " << _pcd_name.second << endl;
        pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
        pcl::io::loadPCDFile<PointType> (_pcd_name.second, *thisKeyFrame);
        cloudKeyFrames.push_back(thisKeyFrame);

        num_pcd_loaded++;
        if(num_pcd_loaded >= nodes_.size()) {
            break;
        }
    }
    cout << "PCDs are loaded (" << name_ << ")" << endl;
}


void Session::loadSessionDescriptors()
{
    std::string desc_dir = session_dir_path_ + "/RefereeDescriptors/";

    // parse names (sorted by the node index before the comma, so the index in
    // placeIndex is the node index)
    std::map<int, std::string> desc_names;
    for(auto& _desc : fs::directory_iterator(desc_dir))
    {
        std::string desc_name = _desc.path().filename();

        std::stringstream desc_name_stream {desc_name};
        std::string desc_idx_str;
        getline(desc_name_stream, desc_idx_str, ',');

        desc_names.insert(std::make_pair(std::stoi(desc_idx_str), _desc.path()));
    }

    int num_loaded = 0;
    for (auto const& _desc_name: desc_names)
    {
        std::vector<float> range_desc, angle_desc;
        if (!referee_slam::readRefereeDescriptorText(_desc_name.second, range_desc, angle_desc))
            throw std::runtime_error("cannot read " + _desc_name.second);
        placeIndex.add(std::move(range_desc), std::move(angle_desc));

        num_loaded++;
        if(num_loaded >= nodes_.size()) {
            break;
        }
    }
    cout << "ReFeree descriptors are loaded (" << name_ << "): " << placeIndex.size() << endl;
}


void Session::loadSessionGraph() 
{
    std::string posefile_path = session_dir_path_ + "/singlesession_posegraph.g2o";

    std::ifstream posefile_handle (posefile_path);
    std::string strOneLine;
    while (getline(posefile_handle, strOneLine)) 
    {
        G2oLineInfo line_info = splitG2oFileLine(strOneLine);

        // save variables (nodes)
        if( isTwoStringSame(line_info.type, G2oLineInfo::kVertexTypeName) ) {
            Node this_node { line_info.curr_idx, gtsam::Pose3( 
                gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) }; 
            nodes_.insert(std::pair<int, Node>(line_info.curr_idx, this_node)); // giseop for multimap
        }
 
        // save edges 
        if( isTwoStringSame(line_info.type, G2oLineInfo::kEdgeTypeName) ) {
            Edge this_edge { line_info.prev_idx, line_info.curr_idx, gtsam::Pose3( 
                gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) }; 
            edges_.insert(std::pair<int, Edge>(line_info.prev_idx, this_edge)); // giseop for multimap
        }
    }
    
    initKeyPoses();

    // 
    ROS_INFO_STREAM("\033[1;32m Session loaded: " << posefile_path << "\033[0m");
    ROS_INFO_STREAM("\033[1;32m - num nodes: " << nodes_.size() << "\033[0m");
} // loadSessionGraph

