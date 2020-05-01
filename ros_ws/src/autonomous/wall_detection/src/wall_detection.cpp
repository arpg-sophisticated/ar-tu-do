#include "wall_detection.h"

WallDetection::WallDetection() {
  this->m_voxel_subscriber = m_node_handle.subscribe<PointCloud>(
      TOPIC_VOXEL_, 1, &WallDetection::wallDetection_callback, this);

  this->m_wall_publisher =
      m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZI>>(TOPIC_WALLS_, 1);

  this->m_obstacles_publisher =
      m_node_handle.advertise<pcl::PointCloud<pcl::PointXYZI>>(TOPIC_OBSTACLES_,
                                                               1);
}

void WallDetection::wallDetection_callback(
    const PointCloud::ConstPtr &inputVoxels) {

  frameID = inputVoxels->header.frame_id;

  float voxelResolution = 0.2f;

  printf("Cloud: width = %d, height = %d\n", inputVoxels->width,
         inputVoxels->height);

  // std::map<int,std::vector<pcl::PointXYZ>> clustersUsed;

  std::unordered_map<int, std::vector<pcl::PointXYZI> *> clustersUsed;

  for (size_t i = 0; i < inputVoxels->points.size(); i++) {
    // if (clustersUsed.find(inputVoxels->points[i].z)!= clustersUsed.end())
    if (clustersUsed.count(inputVoxels->points[i].intensity) > 0) {

      clustersUsed[inputVoxels->points[i].intensity]->push_back(
          inputVoxels->points[i]);
      // std::cout << "Der Liste wurde ein neues Element hinzugefügt und beträgt
      // "
      //              "nun die Läne: "
      //           << clustersUsed[inputVoxels->points[i].z]->size() << "\n";

    } else {
      // std::cout << "Erster Add der Liste\n";
      // std::vector<pcl::PointXYZI> *tmp = new std::vector<pcl::PointXYZ>();
      // tmp->push_back(pcl::PointXYZ(inputVoxels->points[i].x,
      //                              inputVoxels->points[i].y,
      //                              inputVoxels->points[i].z));
      clustersUsed.insert(
          {inputVoxels->points[i].intensity, inputVoxels->points[i]});

      std::cout << clustersUsed.size() << "\n";
    }

    // std::cout << "Punkt Nr. " << i << " --------- " <<
    // inputVoxels->points[i].x
    //           << "|" << inputVoxels->points[i].y << "|"
    //           << inputVoxels->points[i].z << "\n";
  }

  auto test = voxelMaximaIDs(clustersUsed);

  std::cout << "Hier unsere Lösung: " << test.first << "|" << test.second
            << "\n";

  publishWall(clustersUsed[test.first], clustersUsed[test.second]);

  // this->m_debug_geometry.drawVoxels(0, (clustersUsed[test.first]*) ,
  // voxelResolution,
  //                                   voxelResolution, 1 / 10.0f);
  // for (size_t i = 0; i < clustersUsed.at(2).size(); i++)
  // {
  //   std::cout << clustersUsed.at(2)[0].x << "|" << clustersUsed.at(2)[0].y <<
  //   "|" << clustersUsed.at(2)[0].z <<"\n" ;
  // }

  // BOOST_FOREACH (const pcl::PointXYZ& pt, inputVoxels->points)
  //    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

void WallDetection::publishObstacles(
    std::unordered_map<int, std::vector<pcl::PointXYZI> *> mapClusters,
    std::pair<int, int> wallIDs) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto itr = mapClusters.begin(); itr != mapClusters.end(); ++itr) {
    if (itr->first != wallIDs.first || itr->first != wallIDs.second) {
      for (size_t i = 0; i < itr->second->size(); i++) {
        msg->push_back(itr->second[i]);
      }
    }

    // for (auto itrVector = itr->second->begin(); itrVector !=
    // itr->second->end();
    //      ++itrVector) {
    //   if ((itrVector->y > maxLeft) && (itrVector->x <= Schwellwert) &&
    //       (itrVector->x >= -Schwellwert)) {
    //     maxLeft = itrVector->x;
    //     maxLeftID = itrVector->z;
    //   }
    //   if ((itrVector->y < maxRight) && (itrVector->x <= Schwellwert) &&
    //       (itrVector->x >= -Schwellwert)) {
    //     maxRight = itrVector->x;
    //     maxRightID = itrVector->z;
    //   }
    // }
  }
}

void WallDetection::publishWall(std::vector<pcl::PointXYZI> *wallLeft,
                                std::vector<pcl::PointXYZI> *wallRight) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);
  msg->header.frame_id = frameID;
  msg->height = 1;
  msg->width = wallLeft->size() + wallRight->size();
  // msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  for (size_t i = 0; i < wallLeft->size(); i++) {
    pcl::PointXYZI tmp;
    tmp.x = (*wallLeft)[i].x;
    tmp.y = (*wallLeft)[i].y;
    tmp.z = (*wallLeft)[i].z;
    tmp.intensity = 1;

    msg->points.push_back(tmp);

    // pcl::PointXYZI(wallLeft[i]., voxels[i].y, voxels[i].clusterID));
  }
  for (size_t i = 0; i < wallRight->size(); i++) {
    pcl::PointXYZI tmp;
    tmp.x = (*wallRight)[i].x;
    tmp.y = (*wallRight)[i].y;
    tmp.z = (*wallRight)[i].z;
    tmp.intensity = 0;

    msg->points.push_back(tmp);

    // pcl::PointXYZI(wallLeft[i]., voxels[i].y, voxels[i].clusterID));
  }
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  m_wall_publisher.publish(msg);
}

std::pair<int, int> WallDetection::voxelMaximaIDs(
    std::unordered_map<int, std::vector<pcl::PointXYZI> *> mapToCheck) {
  float maxLeft = 0;
  float maxRight = 0;
  int maxLeftID = -1;
  int maxRightID = -1;

  int Schwellwert = 3;

  for (auto itr = mapToCheck.begin(); itr != mapToCheck.end(); ++itr) {
    for (auto itrVector = itr->second->begin(); itrVector != itr->second->end();
         ++itrVector) {
      if ((itrVector->y > maxLeft) && (itrVector->x <= Schwellwert) &&
          (itrVector->x >= -Schwellwert)) {
        maxLeft = itrVector->x;
        maxLeftID = itrVector->z;
      }
      if ((itrVector->y < maxRight) && (itrVector->x <= Schwellwert) &&
          (itrVector->x >= -Schwellwert)) {
        maxRight = itrVector->x;
        maxRightID = itrVector->z;
      }
    }
  }
  return std::pair<int, int>(maxLeftID, maxRightID);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_detection");
  WallDetection wallDetection;
  ros::spin();
  return EXIT_SUCCESS;
}