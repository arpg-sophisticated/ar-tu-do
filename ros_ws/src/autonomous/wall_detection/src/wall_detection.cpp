#include "wall_detection.h"

WallDetection::WallDetection()
    : m_debug_geometry(m_node_handle, TOPIC_VISUALIZATION_REGION, LIDAR_FRAME) {
  this->m_voxel_subscriber = m_node_handle.subscribe<PointCloud>(
      TOPIC_VOXEL_, 1, &WallDetection::wallDetection_callback, this);
}

void WallDetection::wallDetection_callback(
    const PointCloud::ConstPtr &inputVoxels) {
  float voxelResolution = 0.2f;
  this->m_debug_geometry.drawVoxels(0, inputVoxels, voxelResolution,
                                    voxelResolution, 1 / 10.0f);
  printf("Cloud: width = %d, height = %d\n", inputVoxels->width,
         inputVoxels->height);

  // std::map<int,std::vector<pcl::PointXYZ>> clustersUsed;

  std::unordered_map<int, std::vector<pcl::PointXYZ> *> clustersUsed;

  for (size_t i = 0; i < inputVoxels->points.size(); i++) {
    // if (clustersUsed.find(inputVoxels->points[i].z)!= clustersUsed.end())
    if (clustersUsed.count(inputVoxels->points[i].z) > 0) {
      // for (auto itr = clustersUsed.find(i); itr != clustersUsed.end(); itr++)
      // {
      //   std::cout << itr->first
      //      << "\t" << itr->second->size() << '\n';
      //   itr->second->push_back( pcl::PointXYZ(inputVoxels->points[i].x,
      //       inputVoxels->points[i].y,inputVoxels->points[i].z));

      //   std::cout << "Erster if Fall\n";
      //   std::cout << "Der Liste wurde ein neues Element hinzugefügt und
      //   beträgt nun die Läne: "<< itr->second.size() <<"\n";

      // }
      clustersUsed[inputVoxels->points[i].z]->push_back(
          pcl::PointXYZ(inputVoxels->points[i].x, inputVoxels->points[i].y,
                        inputVoxels->points[i].z));
      std::cout << "Der Liste wurde ein neues Element hinzugefügt und beträgt "
                   "nun die Läne: "
                << clustersUsed[inputVoxels->points[i].z]->size() << "\n";

    } else {
      std::cout << "Erster Add der Liste\n";
      std::vector<pcl::PointXYZ> *tmp = new std::vector<pcl::PointXYZ>();
      tmp->push_back(pcl::PointXYZ(inputVoxels->points[i].x,
                                   inputVoxels->points[i].y,
                                   inputVoxels->points[i].z));
      clustersUsed.insert({inputVoxels->points[i].z, tmp});

      std::cout << clustersUsed.size() << "\n";
    }

    std::cout << "Punkt Nr. " << i << " --------- " << inputVoxels->points[i].x
              << "|" << inputVoxels->points[i].y << "|"
              << inputVoxels->points[i].z << "\n";
  }

  std::cout << clustersUsed.size() << "mit "
            << "\n";

  // for (size_t i = 0; i < clustersUsed.at(2).size(); i++)
  // {
  //   std::cout << clustersUsed.at(2)[0].x << "|" << clustersUsed.at(2)[0].y <<
  //   "|" << clustersUsed.at(2)[0].z <<"\n" ;
  // }

  // BOOST_FOREACH (const pcl::PointXYZ& pt, inputVoxels->points)
  //    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

// PointCloud WallDetection::pointCloudOfID(const PointCloud::ConstPtr&
// inputVoxels,int ID){
//   PointCloud::Ptr msg (new PointCloud);
//   msg->header.frame_id = "some_tf_frame";
//   msg->height = 1;

//  // msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

//   for(size_t i=0; i < inputVoxels->width;i++){
//     if()
//     msg->points.push_back(pcl::PointXYZ(voxels[i].x,voxels[i].y,voxels[i].clusterID));
//   }
//   pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_detection");
  WallDetection wallDetection;
  ros::spin();
  return EXIT_SUCCESS;
}