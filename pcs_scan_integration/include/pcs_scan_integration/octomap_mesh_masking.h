/**
 * @file octomap_mesh_masking.h
 * @brief Masks a mesh based on an octomap
 *
 * @author Matthew Powelson
 * @date Sept 23, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef PCS_SCAN_INTEGRATION_OCTOMAP_MESH_MASKING_H
#define PCS_SCAN_INTEGRATION_OCTOMAP_MESH_MASKING_H

#include <memory>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <octomap/ColorOcTree.h>

#include <tesseract_geometry/geometries.h>

namespace pcs_scan_integration
{
class OctomapMeshMask
{
public:
  OctomapMeshMask();

  void setInputMesh(const pcl::PolygonMeshConstPtr& input_mesh) { input_mesh_ = input_mesh; }

  pcl::PolygonMeshConstPtr getInputMesh() { return input_mesh_; }

  void setOctree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud,
                 const double resolution,
                 const int& lower_limit,
                 const int& upper_limit,
                 const bool& limit_negative)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_r;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_g;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_b;
    {
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(point_cloud);
      pass.setFilterFieldName("r");
      pass.setFilterLimits(static_cast<float>(lower_limit), static_cast<float>(upper_limit));
      pass.setFilterLimitsNegative(limit_negative);
      pass.filter(*cloud_filtered_r);
    }
    {
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(cloud_filtered_r);
      pass.setFilterFieldName("g");
      pass.setFilterLimits(static_cast<float>(lower_limit), static_cast<float>(upper_limit));
      pass.setFilterLimitsNegative(limit_negative);
      pass.filter(*cloud_filtered_g);
    }
    {
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(cloud_filtered_g);
      pass.setFilterFieldName("b");
      pass.setFilterLimits(static_cast<float>(lower_limit), static_cast<float>(upper_limit));
      pass.setFilterLimitsNegative(limit_negative);
      pass.filter(*cloud_filtered_b);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = cloud_filtered_b;

    octree_ = std::make_shared<tesseract_geometry::Octree>(
        *cloud_filtered, resolution, tesseract_geometry::Octree::SubType::BOX, true);
  }

  void setOctree(const tesseract_geometry::Octree::Ptr octree) { octree_ = octree; }

  tesseract_geometry::Octree::Ptr getOctree() { return octree_; }

  bool maskMesh(const tesseract_geometry::Octree::Ptr octree)
  {
    // Do all the processing

    return true;
  }

protected:
  pcl::PolygonMeshConstPtr input_mesh_;
  tesseract_geometry::Octree::Ptr octree_;
};

}  // namespace pcs_scan_integration

#endif
