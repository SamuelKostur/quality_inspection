/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

//https://github.com/PointCloudLibrary/pcl/commit/04ae840eb43c78f0e348c35271932c5d6d350759
//this file is combination of:
//https://github.com/PointCloudLibrary/pcl/blob/04ae840eb43c78f0e348c35271932c5d6d350759/common/include/pcl/common/transforms.h
//and 
//https://github.com/PointCloudLibrary/pcl/blob/04ae840eb43c78f0e348c35271932c5d6d350759/common/include/pcl/common/impl/transforms.hpp
//////////////
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/PointIndices.h>

#if defined(__SSE2__)
#include <xmmintrin.h>
#endif

#if defined(__AVX__)
#include <immintrin.h>
#endif

namespace newPcl
{

  namespace detail
  {

    /** A helper struct to apply an SO3 or SE3 transform to a 3D point.
      * Supports single and double precision transform matrices. */
    template<typename Scalar>
    struct Transformer
    {
      const Eigen::Matrix<Scalar, 4, 4>& tf;

      /** Construct a transformer object.
        * The transform matrix is captured by const reference. Make sure that it does not go out of scope before this
        * object does. */
      Transformer (const Eigen::Matrix<Scalar, 4, 4>& transform) : tf (transform) { };

      /** Apply SO3 transform (top-left corner of the transform matrix).
        * \param[in] src input 3D point (pointer to 3 floats)
        * \param[out] tgt output 3D point (pointer to 4 floats), can be the same as input. The fourth element is set to 0. */
      void so3 (const float* src, float* tgt) const
      {
        const Scalar p[3] = { src[0], src[1], src[2] };  // need this when src == tgt
        tgt[0] = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2]);
        tgt[1] = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2]);
        tgt[2] = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2]);
        tgt[3] = 0;
      }

      /** Apply SE3 transform.
        * \param[in] src input 3D point (pointer to 3 floats)
        * \param[out] tgt output 3D point (pointer to 4 floats), can be the same as input. The fourth element is set to 1. */
      void se3 (const float* src, float* tgt) const
      {
        const Scalar p[3] = { src[0], src[1], src[2] };  // need this when src == tgt
        tgt[0] = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2] + tf (0, 3));
        tgt[1] = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2] + tf (1, 3));
        tgt[2] = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2] + tf (2, 3));
        tgt[3] = 1;
      }
    };

#if defined(__SSE2__)

    /** Optimized version for single-precision transforms using SSE2 intrinsics. */
    template<>
    struct Transformer<float>
    {
      /// Columns of the transform matrix stored in XMM registers.
      __m128 c[4];

      Transformer(const Eigen::Matrix4f& tf)
      {
        for (size_t i = 0; i < 4; ++i)
          c[i] = _mm_load_ps (tf.col (i).data ());
      }

      void so3 (const float* src, float* tgt) const
      {
        __m128 p0 = _mm_mul_ps (_mm_load_ps1 (&src[0]), c[0]);
        __m128 p1 = _mm_mul_ps (_mm_load_ps1 (&src[1]), c[1]);
        __m128 p2 = _mm_mul_ps (_mm_load_ps1 (&src[2]), c[2]);
        _mm_store_ps (tgt, _mm_add_ps(p0, _mm_add_ps(p1, p2)));
      }

      void se3 (const float* src, float* tgt) const
      {
        __m128 p0 = _mm_mul_ps (_mm_load_ps1 (&src[0]), c[0]);
        __m128 p1 = _mm_mul_ps (_mm_load_ps1 (&src[1]), c[1]);
        __m128 p2 = _mm_mul_ps (_mm_load_ps1 (&src[2]), c[2]);
        _mm_store_ps (tgt, _mm_add_ps(p0, _mm_add_ps(p1, _mm_add_ps(p2, c[3]))));
      }
    };

#if not defined(__AVX__)

    /** Optimized version for double-precision transform using SSE2 intrinsics. */
    template<>
    struct Transformer<double>
    {
      /// Columns of the transform matrix stored in XMM registers.
      __m128d c[4][2];

      Transformer(const Eigen::Matrix4d& tf)
      {
        for (size_t i = 0; i < 4; ++i)
        {
          c[i][0] = _mm_load_pd (tf.col (i).data () + 0);
          c[i][1] = _mm_load_pd (tf.col (i).data () + 2);
        }
      }

      void so3 (const float* src, float* tgt) const
      {
        __m128d xx = _mm_cvtps_pd (_mm_load_ps1 (&src[0]));
        __m128d p0 = _mm_mul_pd (xx, c[0][0]);
        __m128d p1 = _mm_mul_pd (xx, c[0][1]);

        for (size_t i = 1; i < 3; ++i)
        {
          __m128d vv = _mm_cvtps_pd (_mm_load_ps1 (&src[i]));
          p0 = _mm_add_pd (_mm_mul_pd (vv, c[i][0]), p0);
          p1 = _mm_add_pd (_mm_mul_pd (vv, c[i][1]), p1);
        }

        _mm_store_ps (tgt, _mm_movelh_ps (_mm_cvtpd_ps (p0), _mm_cvtpd_ps (p1)));
      }

      void se3 (const float* src, float* tgt) const
      {
        __m128d p0 = c[3][0];
        __m128d p1 = c[3][1];

        for (size_t i = 0; i < 3; ++i)
        {
          __m128d vv = _mm_cvtps_pd (_mm_load_ps1 (&src[i]));
          p0 = _mm_add_pd (_mm_mul_pd (vv, c[i][0]), p0);
          p1 = _mm_add_pd (_mm_mul_pd (vv, c[i][1]), p1);
        }

        _mm_store_ps (tgt, _mm_movelh_ps (_mm_cvtpd_ps (p0), _mm_cvtpd_ps (p1)));
      }

    };

#else

  /** Optimized version for double-precision transform using AVX intrinsics. */
  template<>
  struct Transformer<double>
  {
    __m256d c[4];

    Transformer(const Eigen::Matrix4d& tf)
    {
      for (size_t i = 0; i < 4; ++i)
        c[i] = _mm256_load_pd (tf.col (i).data ());
    }

    void so3 (const float* src, float* tgt) const
    {
      __m256d p0 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[0])), c[0]);
      __m256d p1 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[1])), c[1]);
      __m256d p2 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[2])), c[2]);
      _mm_store_ps (tgt, _mm256_cvtpd_ps (_mm256_add_pd(p0, _mm256_add_pd(p1, p2))));
    }

    void se3 (const float* src, float* tgt) const
    {
      __m256d p0 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[0])), c[0]);
      __m256d p1 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[1])), c[1]);
      __m256d p2 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[2])), c[2]);
      _mm_store_ps (tgt, _mm256_cvtpd_ps (_mm256_add_pd(p0, _mm256_add_pd(p1, _mm256_add_pd(p2, c[3])))));
    }

  };

#endif
#endif

  }

}
namespace newPcl{  
  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z) should be copied into the new transformed cloud
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                       bool copy_all_fields = true);

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Affine3f &transform,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, float> (cloud_in, cloud_out, transform, copy_all_fields));
  }

  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z) should be copied into the new transformed cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                       bool copy_all_fields = true);

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Affine3f &transform,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, float> (cloud_in, indices, cloud_out, transform, copy_all_fields));
  }

  /** \brief Apply an affine transform defined by an Eigen Transform
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z) should be copied into the new transformed cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const pcl::PointIndices &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, Scalar> (cloud_in, indices.indices, cloud_out, transform, copy_all_fields));
  }

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const pcl::PointIndices &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Affine3f &transform,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, float> (cloud_in, indices, cloud_out, transform, copy_all_fields));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z, normal_x, normal_y, normal_z) should be copied into the new
    * transformed cloud
    * \note Can be used with cloud_in equal to cloud_out
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                                  bool copy_all_fields = true);

  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Affine3f &transform,
                                  bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloudWithNormals<PointT, float> (cloud_in, cloud_out, transform, copy_all_fields));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z, normal_x, normal_y, normal_z) should be copied into the new
    * transformed cloud
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const std::vector<int> &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                                  bool copy_all_fields = true);

  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const std::vector<int> &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Affine3f &transform,
                                  bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloudWithNormals<PointT, float> (cloud_in, indices, cloud_out, transform, copy_all_fields));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z, normal_x, normal_y, normal_z) should be copied into the new
    * transformed cloud
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const pcl::PointIndices &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                                  bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloudWithNormals<PointT, Scalar> (cloud_in, indices.indices, cloud_out, transform, copy_all_fields));
  }


  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const pcl::PointIndices &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Affine3f &transform,
                                  bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloudWithNormals<PointT, float> (cloud_in, indices, cloud_out, transform, copy_all_fields));
  }

  /** \brief Apply a rigid transform defined by a 4x4 matrix
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform a rigid transformation 
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z) should be copied into the new transformed cloud
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix<Scalar, 4, 4> &transform,
                       bool copy_all_fields = true)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (newPcl::transformPointCloud<PointT, Scalar> (cloud_in, cloud_out, t, copy_all_fields));
  }

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix4f &transform,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, float> (cloud_in, cloud_out, transform, copy_all_fields));
  }

  /** \brief Apply a rigid transform defined by a 4x4 matrix
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform a rigid transformation 
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z) should be copied into the new transformed cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix<Scalar, 4, 4> &transform,
                       bool copy_all_fields = true)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (newPcl::transformPointCloud<PointT, Scalar> (cloud_in, indices, cloud_out, t, copy_all_fields));
  }

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const std::vector<int> &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix4f &transform,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, float> (cloud_in, indices, cloud_out, transform, copy_all_fields));
  }

  /** \brief Apply a rigid transform defined by a 4x4 matrix
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform a rigid transformation 
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z) should be copied into the new transformed cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const pcl::PointIndices &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix<Scalar, 4, 4> &transform,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, Scalar> (cloud_in, indices.indices, cloud_out, transform, copy_all_fields));
  }

  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       const pcl::PointIndices &indices, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix4f &transform,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, float> (cloud_in, indices, cloud_out, transform, copy_all_fields));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z, normal_x, normal_y, normal_z) should be copied into the new
    * transformed cloud
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix<Scalar, 4, 4> &transform,
                                  bool copy_all_fields = true)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (newPcl::transformPointCloudWithNormals<PointT, Scalar> (cloud_in, cloud_out, t, copy_all_fields));
  }


  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix4f &transform,
                                  bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloudWithNormals<PointT, float> (cloud_in, cloud_out, transform, copy_all_fields));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z, normal_x, normal_y, normal_z) should be copied into the new
    * transformed cloud
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const std::vector<int> &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix<Scalar, 4, 4> &transform,
                                  bool copy_all_fields = true)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (newPcl::transformPointCloudWithNormals<PointT, Scalar> (cloud_in, indices, cloud_out, t, copy_all_fields));
  }


  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const std::vector<int> &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix4f &transform,
                                  bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloudWithNormals<PointT, float> (cloud_in, indices, cloud_out, transform, copy_all_fields));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[in] indices the set of point indices to use from the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] transform an affine transformation (typically a rigid transformation)
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z, normal_x, normal_y, normal_z) should be copied into the new
    * transformed cloud
    * \note Can be used with cloud_in equal to cloud_out
    * \ingroup common
    */
  template <typename PointT, typename Scalar> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const pcl::PointIndices &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix<Scalar, 4, 4> &transform,
                                  bool copy_all_fields = true)
  {
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (transform);
    return (newPcl::transformPointCloudWithNormals<PointT, Scalar> (cloud_in, indices, cloud_out, t, copy_all_fields));
  }


  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  const pcl::PointIndices &indices, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix4f &transform,
                                  bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloudWithNormals<PointT, float> (cloud_in, indices, cloud_out, transform, copy_all_fields));
  }

  /** \brief Apply a rigid transform defined by a 3D offset and a quaternion
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] offset the translation component of the rigid transformation
    * \param[in] rotation the rotation component of the rigid transformation
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z) should be copied into the new transformed cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Matrix<Scalar, 3, 1> &offset, 
                       const Eigen::Quaternion<Scalar> &rotation,
                       bool copy_all_fields = true);

  template <typename PointT> inline void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                       pcl::PointCloud<PointT> &cloud_out, 
                       const Eigen::Vector3f &offset, 
                       const Eigen::Quaternionf &rotation,
                       bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloud<PointT, float> (cloud_in, cloud_out, offset, rotation, copy_all_fields));
  }

  /** \brief Transform a point cloud and rotate its normals using an Eigen transform.
    * \param[in] cloud_in the input point cloud
    * \param[out] cloud_out the resultant output point cloud
    * \param[in] offset the translation component of the rigid transformation
    * \param[in] rotation the rotation component of the rigid transformation
    * \param[in] copy_all_fields flag that controls whether the contents of the fields
    * (other than x, y, z, normal_x, normal_y, normal_z) should be copied into the new
    * transformed cloud
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Matrix<Scalar, 3, 1> &offset, 
                                  const Eigen::Quaternion<Scalar> &rotation,
                                  bool copy_all_fields = true);

  template <typename PointT> void 
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                  pcl::PointCloud<PointT> &cloud_out, 
                                  const Eigen::Vector3f &offset, 
                                  const Eigen::Quaternionf &rotation,
                                  bool copy_all_fields = true)
  {
    return (newPcl::transformPointCloudWithNormals<PointT, float> (cloud_in, cloud_out, offset, rotation, copy_all_fields));
  }

  /** \brief Transform a point with members x,y,z
    * \param[in] point the point to transform
    * \param[out] transform the transformation to apply
    * \return the transformed point
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline PointT
  transformPoint (const PointT &point, 
                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);
  
  template <typename PointT> inline PointT
  transformPoint (const PointT &point, 
                  const Eigen::Affine3f &transform)
  {
    return (transformPoint<PointT, float> (point, transform));
  }

  /** \brief Transform a point with members x,y,z,normal_x,normal_y,normal_z
    * \param[in] point the point to transform
    * \param[out] transform the transformation to apply
    * \return the transformed point
    * \ingroup common
    */
  template <typename PointT, typename Scalar> inline PointT
  transformPointWithNormal (const PointT &point, 
                  const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);
  
  template <typename PointT> inline PointT
  transformPointWithNormal (const PointT &point, 
                  const Eigen::Affine3f &transform)
  {
    return (transformPointWithNormal<PointT, float> (point, transform));
  }

  /** \brief Calculates the principal (PCA-based) alignment of the point cloud
    * \param[in] cloud the input point cloud
    * \param[out] transform the resultant transform
    * \return the ratio lambda1/lambda2 or lambda2/lambda3, whatever is closer to 1.
    * \note If the return value is close to one then the transformation might be not unique -> two principal directions have
    * almost same variance (extend)
    */
  template <typename PointT, typename Scalar> inline double
  getPrincipalTransformation (const pcl::PointCloud<PointT> &cloud, 
                              Eigen::Transform<Scalar, 3, Eigen::Affine> &transform);

  template <typename PointT> inline double
  getPrincipalTransformation (const pcl::PointCloud<PointT> &cloud, 
                              Eigen::Affine3f &transform)
  {
    return (getPrincipalTransformation<PointT, float> (cloud, transform));
  }
}
namespace newPcl{
////
///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> void
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                            pcl::PointCloud<PointT> &cloud_out,
                            const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                            bool copy_all_fields)
  {
    if (&cloud_in != &cloud_out)
    {
      cloud_out.header   = cloud_in.header;
      cloud_out.is_dense = cloud_in.is_dense;
      cloud_out.width    = cloud_in.width;
      cloud_out.height   = cloud_in.height;
      cloud_out.points.reserve (cloud_in.points.size ());
      if (copy_all_fields)
        cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
      else
        cloud_out.points.resize (cloud_in.points.size ());
      cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
      cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
    }

    newPcl::detail::Transformer<Scalar> tf (transform.matrix ());
    if (cloud_in.is_dense)
    {
      // If the dataset is dense, simply transform it!
      for (size_t i = 0; i < cloud_out.points.size (); ++i)
        tf.se3 (cloud_in[i].data, cloud_out[i].data);
    }
    else
    {
      // Dataset might contain NaNs and Infs, so check for them first,
      // otherwise we get errors during the multiplication (?)
      for (size_t i = 0; i < cloud_out.points.size (); ++i)
      {
        if (!pcl_isfinite (cloud_in.points[i].x) || 
            !pcl_isfinite (cloud_in.points[i].y) || 
            !pcl_isfinite (cloud_in.points[i].z))
          continue;
        tf.se3 (cloud_in[i].data, cloud_out[i].data);
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> void
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                            const std::vector<int> &indices, 
                            pcl::PointCloud<PointT> &cloud_out,
                            const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                            bool copy_all_fields)
  {
    size_t npts = indices.size ();
    // In order to transform the data, we need to remove NaNs
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = static_cast<int> (npts);
    cloud_out.height   = 1;
    cloud_out.points.resize (npts);
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_      = cloud_in.sensor_origin_;

    newPcl::detail::Transformer<Scalar> tf (transform.matrix ());
    if (cloud_in.is_dense)
    {
      // If the dataset is dense, simply transform it!
      for (size_t i = 0; i < npts; ++i)
      {
        // Copy fields first, then transform xyz data
        if (copy_all_fields)
          cloud_out.points[i] = cloud_in.points[indices[i]];
        tf.se3 (cloud_in[indices[i]].data, cloud_out[i].data);
      }
    }
    else
    {
      // Dataset might contain NaNs and Infs, so check for them first,
      // otherwise we get errors during the multiplication (?)
      for (size_t i = 0; i < npts; ++i)
      {
        if (copy_all_fields)
          cloud_out.points[i] = cloud_in.points[indices[i]];
        if (!pcl_isfinite (cloud_in.points[indices[i]].x) || 
            !pcl_isfinite (cloud_in.points[indices[i]].y) || 
            !pcl_isfinite (cloud_in.points[indices[i]].z))
          continue;
        tf.se3 (cloud_in[indices[i]].data, cloud_out[i].data);
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> void
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                      pcl::PointCloud<PointT> &cloud_out,
                                      const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                                      bool copy_all_fields)
  {
    if (&cloud_in != &cloud_out)
    {
      // Note: could be replaced by cloud_out = cloud_in
      cloud_out.header   = cloud_in.header;
      cloud_out.width    = cloud_in.width;
      cloud_out.height   = cloud_in.height;
      cloud_out.is_dense = cloud_in.is_dense;
      cloud_out.points.reserve (cloud_out.points.size ());
      if (copy_all_fields)
        cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
      else
        cloud_out.points.resize (cloud_in.points.size ());
      cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
      cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
    }

    newPcl::detail::Transformer<Scalar> tf (transform.matrix ());
    // If the data is dense, we don't need to check for NaN
    if (cloud_in.is_dense)
    {
      for (size_t i = 0; i < cloud_out.points.size (); ++i)
      {
        tf.se3 (cloud_in[i].data, cloud_out[i].data);
        tf.so3 (cloud_in[i].data_n, cloud_out[i].data_n);
      }
    }
    // Dataset might contain NaNs and Infs, so check for them first.
    else
    {
      for (size_t i = 0; i < cloud_out.points.size (); ++i)
      {
        if (!pcl_isfinite (cloud_in.points[i].x) || 
            !pcl_isfinite (cloud_in.points[i].y) || 
            !pcl_isfinite (cloud_in.points[i].z))
          continue;
        tf.se3 (cloud_in[i].data, cloud_out[i].data);
        tf.so3 (cloud_in[i].data_n, cloud_out[i].data_n);
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> void
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                      const std::vector<int> &indices, 
                                      pcl::PointCloud<PointT> &cloud_out,
                                      const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform,
                                      bool copy_all_fields)
  {
    size_t npts = indices.size ();
    // In order to transform the data, we need to remove NaNs
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = static_cast<int> (npts);
    cloud_out.height   = 1;
    cloud_out.points.resize (npts);
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_      = cloud_in.sensor_origin_;

    newPcl::detail::Transformer<Scalar> tf (transform.matrix ());
    // If the data is dense, we don't need to check for NaN
    if (cloud_in.is_dense)
    {
      for (size_t i = 0; i < cloud_out.points.size (); ++i)
      {
        // Copy fields first, then transform
        if (copy_all_fields)
          cloud_out.points[i] = cloud_in.points[indices[i]];
        tf.se3 (cloud_in[indices[i]].data, cloud_out[i].data);
        tf.so3 (cloud_in[indices[i]].data_n, cloud_out[i].data_n);
      }
    }
    // Dataset might contain NaNs and Infs, so check for them first.
    else
    {
      for (size_t i = 0; i < cloud_out.points.size (); ++i)
      {
        // Copy fields first, then transform
        if (copy_all_fields)
          cloud_out.points[i] = cloud_in.points[indices[i]];

        if (!pcl_isfinite (cloud_in.points[indices[i]].x) || 
            !pcl_isfinite (cloud_in.points[indices[i]].y) || 
            !pcl_isfinite (cloud_in.points[indices[i]].z))
          continue;

        tf.se3 (cloud_in[indices[i]].data, cloud_out[i].data);
        tf.so3 (cloud_in[indices[i]].data_n, cloud_out[i].data_n);
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> inline void
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                            pcl::PointCloud<PointT> &cloud_out,
                            const Eigen::Matrix<Scalar, 3, 1> &offset, 
                            const Eigen::Quaternion<Scalar> &rotation,
                            bool copy_all_fields)
  {
    Eigen::Translation<Scalar, 3> translation (offset);
    // Assemble an Eigen Transform
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (translation * rotation);
    transformPointCloud (cloud_in, cloud_out, t, copy_all_fields);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> inline void
  transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                      pcl::PointCloud<PointT> &cloud_out,
                                      const Eigen::Matrix<Scalar, 3, 1> &offset, 
                                      const Eigen::Quaternion<Scalar> &rotation,
                                      bool copy_all_fields)
  {
    Eigen::Translation<Scalar, 3> translation (offset);
    // Assemble an Eigen Transform
    Eigen::Transform<Scalar, 3, Eigen::Affine> t (translation * rotation);
    transformPointCloudWithNormals (cloud_in, cloud_out, t, copy_all_fields);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> inline PointT
  transformPoint (const PointT &point, 
                      const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
  {
    PointT ret = point;
    newPcl::detail::Transformer<Scalar> tf (transform.matrix ());
    tf.se3 (point.data, ret.data);
    return (ret);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> inline PointT
  transformPointWithNormal (const PointT &point, 
                      const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
  {
    PointT ret = point;
    newPcl::detail::Transformer<Scalar> tf (transform.matrix ());
    tf.se3 (point.data, ret.data);
    tf.so3 (point.data_n, ret.data_n);
    return (ret);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointT, typename Scalar> double
  getPrincipalTransformation (const pcl::PointCloud<PointT> &cloud, 
                                  Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
  {
    EIGEN_ALIGN16 Eigen::Matrix<Scalar, 3, 3> covariance_matrix;
    Eigen::Matrix<Scalar, 4, 1> centroid;
    
    pcl::computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid);

    EIGEN_ALIGN16 Eigen::Matrix<Scalar, 3, 3> eigen_vects;
    Eigen::Matrix<Scalar, 3, 1> eigen_vals;
    pcl::eigen33 (covariance_matrix, eigen_vects, eigen_vals);

    double rel1 = eigen_vals.coeff (0) / eigen_vals.coeff (1);
    double rel2 = eigen_vals.coeff (1) / eigen_vals.coeff (2);
    
    transform.translation () = centroid.head (3);
    transform.linear () = eigen_vects;
    
    return (std::min (rel1, rel2));
  }
}

