/**************************************************************************
 * VIG-Init
 *
 * Copyright SenseTime. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************/

#pragma once

#include <beam_calibration/ConvertCameraModel.h>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/opencv.hpp>
#include <slamtools/common.h>
#include <slamtools/state.h>
#include <string>

class PoissonKeypointFilter {
public:
  PoissonKeypointFilter(double x_min, double x_max, double y_min, double y_max,
                        double radius) {
    m_x_min = x_min;
    m_x_max = x_max;
    m_y_min = y_min;
    m_y_max = y_max;
    m_radius = radius;
    m_radius_squared = radius * radius;
    m_grid_size = radius / sqrt(2);
    m_grid_width = (int)((m_x_max - m_x_min) / m_grid_size) + 1;
    m_grid_height = (int)((m_y_max - m_y_min) / m_grid_size) + 1;
    clear();
  }

  void clear() {
    std::vector<size_t>(m_grid_width * m_grid_height, nil()).swap(m_grid);
    m_points.clear();
  }

  void set_points(const std::vector<Eigen::Vector2d>& points) {
    for (const Eigen::Vector2d& p : points) {
      int ix, iy;
      to_icoord(p, ix, iy);
      m_grid[ix + iy * m_grid_width] = m_points.size();
      m_points.push_back(p);
    }
  }

  void filter(std::vector<Eigen::Vector2d>& points) {
    size_t n_points_before = m_points.size();
    for (const Eigen::Vector2d& p : points) {
      int ix, iy;
      if (test_point(p, ix, iy)) {
        m_grid[ix + iy * m_grid_width] = m_points.size();
        m_points.push_back(p);
      }
    }
    std::vector<Eigen::Vector2d>(m_points.begin() + n_points_before,
                                 m_points.end())
        .swap(points);
  }

private:
  void to_icoord(const Eigen::Vector2d& p, int& ix, int& iy) const {
    ix = int(floor((p.x() - m_x_min) / m_grid_size));
    iy = int(floor((p.y() - m_y_min) / m_grid_size));
  }

  bool test_point(const Eigen::Vector2d& p, int& ix, int& iy) const {
    if (p.x() < m_x_min || p.x() > m_x_max) return false;
    if (p.y() < m_y_min || p.y() > m_y_max) return false;
    to_icoord(p, ix, iy);
    int x_extent_begin = std::max(ix - 2, 0),
        x_extent_end = std::min(ix + 2, m_grid_width - 1);
    int y_extent_begin = std::max(iy - 2, 0),
        y_extent_end = std::min(iy + 2, m_grid_height - 1);
    for (int y = y_extent_begin; y <= y_extent_end; ++y) {
      for (int x = x_extent_begin; x <= x_extent_end; ++x) {
        size_t nbr = m_grid[x + y * m_grid_width];
        if (nbr != nil()) {
          if ((p - m_points[nbr]).squaredNorm() < m_radius_squared) {
            return false;
          }
        }
      }
    }
    return true;
  }

  double m_x_min;
  double m_x_max;
  double m_y_min;
  double m_y_max;
  double m_radius;
  double m_radius_squared;
  double m_grid_size;
  int m_grid_width;
  int m_grid_height;
  std::vector<size_t> m_grid;
  std::vector<Eigen::Vector2d> m_points;
};

static std::vector<cv::Point2f>
    to_opencv(const std::vector<Eigen::Vector2d>& v) {
  std::vector<cv::Point2f> r(v.size());
  for (size_t i = 0; i < v.size(); ++i) {
    r[i].x = v[i].x();
    r[i].y = v[i].y();
  }
  return r;
}

static std::vector<Eigen::Vector2d>
    from_opencv(const std::vector<cv::Point2f>& v) {
  std::vector<Eigen::Vector2d> r(v.size());
  for (size_t i = 0; i < v.size(); ++i) {
    r[i].x() = v[i].x;
    r[i].y() = v[i].y;
  }
  return r;
}

class OpenCvImage : public Image {
public:
  OpenCvImage(cv::Mat img) { image = img; }

  void detect_keypoints(std::vector<Eigen::Vector2d>& keypoints,
                        size_t max_points = 100) const override {
    std::vector<cv::KeyPoint> cvkeypoints;
    gftt()->detect(image, cvkeypoints);

    if (cvkeypoints.size() > 0) {
      std::sort(cvkeypoints.begin(), cvkeypoints.end(),
                [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                  return a.response > b.response;
                });
      std::vector<Eigen::Vector2d> new_keypoints;
      for (size_t i = 0; i < cvkeypoints.size(); ++i) {
        new_keypoints.emplace_back(cvkeypoints[i].pt.x, cvkeypoints[i].pt.y);
      }
      PoissonKeypointFilter filter(20, image.cols - 20, 20, image.rows - 20,
                                   20.0);
      filter.set_points(keypoints);
      filter.filter(new_keypoints);

      keypoints.insert(keypoints.end(), new_keypoints.begin(),
                       new_keypoints.end());
    }
  }

  void track_keypoints(const Image* next_image,
                       const std::vector<Eigen::Vector2d>& curr_keypoints,
                       std::vector<Eigen::Vector2d>& next_keypoints,
                       std::vector<char>& result_status) const override {
    std::vector<cv::Point2f> curr_cvpoints = to_opencv(curr_keypoints);
    std::vector<cv::Point2f> next_cvpoints;
    if (next_keypoints.size() > 0) {
      next_cvpoints = to_opencv(next_keypoints);
    } else {
      next_keypoints.resize(curr_keypoints.size());
      next_cvpoints = curr_cvpoints;
    }

    const OpenCvImage* next_cvimage =
        dynamic_cast<const OpenCvImage*>(next_image);

    result_status.resize(curr_keypoints.size(), 0);
    if (next_cvimage && curr_cvpoints.size() > 0) {
      cv::Mat cvstatus, cverr;
      cv::calcOpticalFlowPyrLK(
          image, next_cvimage->image, curr_cvpoints, next_cvpoints, cvstatus,
          cverr); // Size(21, 21), 3, TermCriteria(TermCriteria::COUNT +
                  // TermCriteria::EPS, 30, 0.01), OPTFLOW_USE_INITIAL_FLOW);
      for (size_t i = 0; i < next_cvpoints.size(); ++i) {
        result_status[i] = cvstatus.at<unsigned char>(i);
        if (next_cvpoints[i].x < 20 || next_cvpoints[i].x >= image.cols - 20 ||
            next_cvpoints[i].y < 20 || next_cvpoints[i].y >= image.rows - 20) {
          result_status[i] = 0;
        }
      }
    }

    std::vector<size_t> l;
    std::vector<cv::Point2f> p, q;
    for (size_t i = 0; i < result_status.size(); ++i) {
      if (result_status[i] != 0) {
        l.push_back(i);
        p.push_back(curr_cvpoints[i]);
        q.push_back(next_cvpoints[i]);
      }
    }
    if (l.size() >= 8) {
      cv::Mat mask;
      cv::findFundamentalMat(p, q, cv::FM_RANSAC, 1.0, 0.99, mask);
      for (size_t i = 0; i < l.size(); ++i) {
        if (mask.at<unsigned char>(i) == 0) { result_status[l[i]] = 0; }
      }
      for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        if (result_status[i]) {
          next_keypoints[i].x() = next_cvpoints[i].x;
          next_keypoints[i].y() = next_cvpoints[i].y;
        }
      }
    }
  }

  void detect_segments(
      std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>& segments,
      size_t max_segments = 0) const override {
    std::vector<cv::line_descriptor::KeyLine> keylines;
    lsd()->detect(image, keylines, 2, 2);
    segments.resize(keylines.size());
    for (size_t i = 0; i < keylines.size(); ++i) {
      std::get<0>(segments[i]).x() = keylines[i].startPointX;
      std::get<0>(segments[i]).y() = keylines[i].startPointY;
      std::get<1>(segments[i]).x() = keylines[i].endPointX;
      std::get<1>(segments[i]).y() = keylines[i].endPointY;
    }
  }

  void preprocess() { clahe()->apply(image, image); }

  void correct_distortion(
      std::shared_ptr<beam_calibration::UndistortImages> rectifier) {
    image = rectifier->ConvertImage<uchar>(image);
  }

  cv::Mat image;

private:
  static cv::CLAHE* clahe() {
    static cv::Ptr<cv::CLAHE> s_clahe = cv::createCLAHE(6.0);
    return s_clahe.get();
  }

  static cv::line_descriptor::LSDDetector* lsd() {
    static cv::Ptr<cv::line_descriptor::LSDDetector> s_lsd =
        cv::line_descriptor::LSDDetector::createLSDDetector();
    return s_lsd.get();
  }

  static cv::GFTTDetector* gftt() {
    static cv::Ptr<cv::GFTTDetector> s_gftt =
        cv::GFTTDetector::create(1000, 1.0e-3, 20, 3, true);
    return s_gftt.get();
  }

  static cv::FastFeatureDetector* fast() {
    static cv::Ptr<cv::FastFeatureDetector> s_fast =
        cv::FastFeatureDetector::create();
    return s_fast.get();
  }

  static cv::ORB* orb() {
    static cv::Ptr<cv::ORB> s_orb = cv::ORB::create();
    return s_orb.get();
  }
};
