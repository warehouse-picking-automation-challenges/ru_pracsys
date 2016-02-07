/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2013, Vincent Rabaud
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#include <object_recognition_renderer/renderer.h>
#include <object_recognition_renderer/utils.h>

bool view_params_never_updated = true;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RendererIterator::RendererIterator(Renderer *renderer, size_t number_of_random_views_)
    :
      //tesselation_level_(tesselation_level),
      number_of_random_views_(number_of_random_views_),
      index_(0),
      renderer_(renderer),
      angle_min_(0),
      angle_max_(359),
      angle_step_(40),
      angle_(angle_min_),
      radius_min_(0.4),
      radius_max_(0.8),
      radius_step_(0.2),
      radius_(radius_min_)
{
  srand(5);
  view_params_never_updated = true;
  //compute_tesselation_sphere(tesselation_level);
  //index_ = -1;
  //update_view_params();

  //std::cout << "++++++++++++++++ cam_positions_.size() = " << cam_positions_.size() << "\n";
  std::cout << "++++++++++++++++ n_templates() = " << n_templates() << "\n";
  //std::cout << "++++++++++++++++ angle_step_ = " << angle_step_ << "\n";
}

/**
 * @param image_out the RGB image
 * @param depth_out the depth image
 * @param mask_out the mask image
 * @param rect_out the bounding box of the rendered image
 */
void
RendererIterator::render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect_out)
{
  if (isDone())
    return;

  cv::Vec3d t, up;
  view_params(t, up);

  renderer_->lookAt(t(0), t(1), t(2), up(0), up(1), up(2));
  //renderer_->render(image_out, depth_out, mask_out, rect_out);
  renderer_->renderDepthOnly(depth_out, mask_out, rect_out);
  renderer_->renderImageOnly(image_out, rect_out);

}

/**
 * @param image_out the RGB image
 * @param depth_out the depth image
 * @param mask_out the mask image
 * @param rect_out the bounding box of the rendered image
 * @param t the translation vector
 * @param up the up vector of the view point
 */
void
RendererIterator::render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect_out, const cv::Vec3d &t, const cv::Vec3d &up)
{
  renderer_->lookAt(t(0), t(1), t(2), up(0), up(1), up(2));
  renderer_->renderDepthOnly(depth_out, mask_out, rect_out);
  renderer_->renderImageOnly(image_out, rect_out);
}

/**
 * @param depth_out the depth image
 * @param mask_out the mask image
 * @param rect_out the bounding box of the rendered image
 * @param t the translation vector
 * @param up the up vector of the view point
 */
void
RendererIterator::renderDepthOnly(cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect_out, const cv::Vec3d &t, const cv::Vec3d &up)
{
  renderer_->lookAt(t(0), t(1), t(2), up(0), up(1), up(2));
  renderer_->renderDepthOnly(depth_out, mask_out, rect_out);
}

/**
 * @param image_out
 * @param rect_out
 * @param t the translation vector
 * @param up the up vector of the view point
 */
void
RendererIterator::renderImageOnly(cv::Mat &image_out, const cv::Rect &rect_out, const cv::Vec3d &t, const cv::Vec3d &up)
{
  renderer_->lookAt(t(0), t(1), t(2), up(0), up(1), up(2));
  renderer_->renderImageOnly(image_out, rect_out);
}


/**
 * @return the rotation of the camera with respect to the current view point
 */
cv::Matx33d
RendererIterator::R()
{
  cv::Vec3d t, up;
  view_params(t, up);
  normalize_vector(t(0),t(1),t(2));

  // compute the left vector
  cv::Vec3d y;
  y = up.cross(t);
  normalize_vector(y(0),y(1),y(2));

  // re-compute the orthonormal up vector
  up = t.cross(y);
  normalize_vector(up(0), up(1), up(2));

  cv::Mat R_full = (cv::Mat_<double>(3, 3) <<
                    t(0), t(1), t(2),
                    y(0), y(1), y(2),
                    up(0), up(1), up(2));
  cv::Matx33d R = R_full;
  R = R.t();

  return R;
}

/**
 * @return the rotation of the object with respect to the current view point
 */
cv::Matx33d
RendererIterator::R_obj()
{
  cv::Vec3d t, up;
  view_params(t, up);
  normalize_vector(t(0),t(1),t(2));

  // compute the left vector
  cv::Vec3d y;
  y = up.cross(t);
  normalize_vector(y(0),y(1),y(2));

  // re-compute the orthonormal up vector
  up = t.cross(y);
  normalize_vector(up(0), up(1), up(2));

  cv::Mat R_full = (cv::Mat_<double>(3, 3) <<
                    -y(0), -y(1), -y(2),
                    -up(0), -up(1), -up(2),
                    t(0), t(1), t(2)
                    );

  cv::Matx33d R = R_full;
  R = R.t();

  return R.inv();
}

/**
 * @return the translation of the camera with respect to the current view point
 */
cv::Vec3d
RendererIterator::T()
{
  cv::Vec3d t, _up;
  view_params(t, _up);

  return -t;
}

///**
// * @return the total number of templates that will be computed
// */
//size_t
//RendererIterator::n_templates()
//{
//  return (cam_positions_.size() *
//      (1 + (angle_max_ - angle_min_) / angle_step_) *
//      (1 + (radius_max_ - radius_min_) / radius_step_));
//}

/**
 * @return the total number of templates that will be computed
 */
size_t
RendererIterator::n_templates() const
{
  return (number_of_random_views_);
}

//bool
//RendererIterator::isDone()
//{
//  return (index_ >= cam_positions_.size());
//}

bool
RendererIterator::isDone()
{
  return (index_ >= number_of_random_views_);
}

bool
comapare_vectors(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
  float angle_xy1 = std::atan2(v1[1], v1[0]);
  float angle_xy2 = std::atan2(v2[1], v2[0]);
  float angle_xz1 = std::atan2(v1[2], v1[0]);
  float angle_xz2 = std::atan2(v2[2], v2[0]);
  float angle_yz1 = std::atan2(v1[2], v1[1]);
  float angle_yz2 = std::atan2(v2[2], v2[1]);

  if (angle_xy1 < angle_xy2)
  {
    return (true);
  }
  else if (angle_xy1 == angle_xy2)
  {
    if (angle_xz1 < angle_xz2)
    {
      return (true);
    }
    else if (angle_xz1 == angle_xz2)
    {
      if (angle_yz1 < angle_yz2)
      {
        return (true);
      }
      else
        return (false);
    }
    else
      return (false);
  }
  else
    return (false);
}
//
//void
//RendererIterator::compute_tesselation_sphere(int tesselation_level)
//{
//  //create icosahedron
//  vtkSmartPointer<vtkPlatonicSolidSource> ico = vtkSmartPointer<vtkPlatonicSolidSource>::New();
//  ico->SetSolidTypeToIcosahedron();
//  ico->Update();
//
//  //tesselate cells from icosahedron
//  vtkSmartPointer<vtkLoopSubdivisionFilter> subdivide = vtkSmartPointer<vtkLoopSubdivisionFilter>::New();
//  subdivide->SetNumberOfSubdivisions(tesselation_level);
//  subdivide->SetInputConnection (ico->GetOutputPort());
//  subdivide->Update ();
//
//  // Get camera positions
//  vtkPolyData *sphere = subdivide->GetOutput();
//
//  cam_positions_.resize (sphere->GetNumberOfPoints());
//  //std::cout << "\n";
//  double cam_pos[3];
//
//  sphere->GetPoint(0, cam_pos);
//  cam_positions_[0] = Eigen::Vector3f (float (cam_pos[0]), float (cam_pos[1]), float (cam_pos[2]));
//  cam_positions_[0] = cam_positions_[0].normalized();
//  //std::cout << cam_positions_[0][0] << ", " << cam_positions_[0][1] << ", " << cam_positions_[0][2] << "\n";
//
//  float max_cos = 0.0;
//  for (int i = 1; i < sphere->GetNumberOfPoints(); i++)
//  {
//    sphere->GetPoint(i, cam_pos);
//    cam_positions_[i] = Eigen::Vector3f (float (cam_pos[0]), float (cam_pos[1]), float (cam_pos[2]));
//    cam_positions_[i] = cam_positions_[i].normalized();
//    //std::cout << cam_positions_[i][0] << ", " << cam_positions_[i][1] << ", " << cam_positions_[i][2] << "\n";
//
//    float cos_two_vectors = cam_positions_[0].dot(cam_positions_[i]) / (cam_positions_[0].norm() * cam_positions_[i].norm());
//    if (max_cos < cos_two_vectors)
//      max_cos = cos_two_vectors;
//  }
//  std::cout << "++++++++++++++++ angle between two camera views = " << 180.0 * std::acos(max_cos) / CV_PI << "\n";
//  //std::sort(cam_positions_.begin(), cam_positions_.end(), comapare_vectors);
//}

void
rotate_vector_a_around_vector_b(cv::Vec3d &a, cv::Vec3d b, float angle)
{
  // Rodrigues formula for rotating a vector by a given angle around another vector
  // http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

  a = a * std::cos(angle) + b.cross(a) * std::sin(angle) + b * b.dot(a) * (1.0 - std::cos(angle));
}

//
//float
//rand_1()
//{
//  float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5;
//  return (r);
//}
//
//void
//add_noise_to_vector_angle(cv::Vec3d &a, float noise_angle)
//{
//  cv::Vec3d b = cv::Vec3d(rand_1(), rand_1(), rand_1());
//  cv::normalize(b, b, 1.0);
//  rotate_vector_a_around_vector_b(a, b, rand_1() * noise_angle);
//}
//
//void
//RendererIterator::update_view_params()
//{
//  cv::Vec3d cam_pos = cv::Vec3d(cam_positions_[index_][0], cam_positions_[index_][1], cam_positions_[index_][2]);
//
//  cv::Vec3d up = cv::Vec3d(0.0, 1.0, 0.0);
//  if (fabs(cam_pos.dot(up)) == 1) // up cannot be parallel to cam_pos...
//  {
//    up = cam_pos.cross(cv::Vec3d(1.0, 0.0, 0.0));
//    //std::cout << "\n\n=============== Could not find a orthogonal up vector. Changing axis...\n\n";
//  }
//  up = cam_pos.cross(up); // up orthogonal to cam_pos
//  cv::normalize(up, up, 1.0);
//  up = cam_pos.cross(up); // up orthogonal to cam_pos
//
//  float angle_rad = ((angle_  + rand_1() * (angle_step_ / 10.0)) / 180.0) * CV_PI;
//  add_noise_to_vector_angle(cam_pos, angle_step_ / 10.0);
//  rotate_vector_a_around_vector_b(up, cam_pos, angle_rad);
//  cv::Vec3d T = radius_ * cam_pos;
//  //std::cout << "\ncam_pos\n" << cam_pos << "\nup\n" << up << "\nT\n" << T << "\n";
//  //std::cout << "\ncam_pos\n" << cv::norm(cam_pos) << "\nup\n" << cv::norm(up) << "\nT\n" << cv::norm(T) << "\n";
//  T_ = T;
//  up_ = up;
//}

double
rand_open_interval_0_1()
{
  double r;

  r = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  while ((r == 1.0) || (r == 0.0))
    r = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);

  return (r);
}

double
rand_open_interval_1()
{
  double r;

  r = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  while (r == 1.0)
    r = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);

  return (r);
}

double
rand_close_interval_0_1()
{
  double r;

  r = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);

  return (r);
}

double
radius_pdf(double x, double a, double b)
{	// The size of the viewed object decreases by half if the distance doubles -> f(x) = 2^-x
	// pdf (f(x)) from a to b = f(x) / (integral f(x) from a to b)
	// So, for min radius a = 0.6 and max radius b = 1.2:
	// https://www.wolframalpha.com/input/?i=integral+2^-x+%2F+%28%282^-0.6+-+2^-1.2%29+%2F+log+2%29+from+0.6+to+1.2
	return (pow(2.0, -x) / ((pow(2.0, -a) - pow(2.0, -b)) / log(2.0)));
}

double
sample_from_radius_pdf(double min_radius, double max_radius)
{	// http://en.wikipedia.org/wiki/Rejection_sampling
	double x, pdf_x, y;
	double M = radius_pdf(min_radius, min_radius, max_radius); // the maximum value of the pdf occurs at min_radius
	do
	{
		x = min_radius + rand_close_interval_0_1() * (max_radius - min_radius); // sample x in the valid pdf interval [max_radius:min_radius]
		pdf_x = radius_pdf(x, min_radius, max_radius);
		y = M * rand_close_interval_0_1();
	} while (y > pdf_x);

	return (x);
}

void
RendererIterator::update_view_params()
{
  double u = rand_open_interval_1();
  double v = rand_open_interval_0_1();

  double lon = u * 2.0 * CV_PI;     // from 0 to 2.0 * pi [0:2*pi)
  double lat = acos(1.0 - v * 2.0); // from 0 to pi (0:pi)

  // http://en.wikipedia.org/wiki/Spherical_coordinate_system
  double x = sin(lat) * cos(lon);
  double y = sin(lat) * sin(lon);
  double z = cos(lat);

  cv::Vec3d cam_pos = cv::Vec3d(x, y, z);

  cv::Vec3d up = cv::Vec3d(0.0, 1.0, 0.0);
  if (fabs(cam_pos.dot(up)) == 1) // up cannot be parallel to cam_pos...
  {
    up = cam_pos.cross(cv::Vec3d(1.0, 0.0, 0.0));
    //std::cout << "\n\n=============== Could not find a orthogonal up vector. Changing axis...\n\n";
  }
  up = cam_pos.cross(up); // up orthogonal to cam_pos
  cv::normalize(up, up, 1.0);
  up = cam_pos.cross(up); // up orthogonal to cam_pos
  cv::normalize(up, up, 1.0);

  double w = rand_open_interval_1();
  double rotation = 2.0 * CV_PI * w - CV_PI;
  rotate_vector_a_around_vector_b(up, cam_pos, rotation);

  //radius_ = radius_min_ + (radius_max_ - radius_min_) * rand_open_interval_0_1();
  radius_ = sample_from_radius_pdf(radius_min_, radius_max_);
  cv::Vec3d T = radius_ * cam_pos;
//  std::cout << "\ncam_pos\n" << cam_pos << "\nup\n" << up << "\nT\n" << T << "\n";
//  std::cout << "\ncam_pos\n" << cv::norm(cam_pos) << "\nup\n" << cv::norm(up) << "\nT\n@@@" << cv::norm(T) << "\n";
  T_ = T;
  up_ = up;
}

//RendererIterator &
//RendererIterator::operator++()
//{
//  angle_ += angle_step_;
//  if (angle_ >= angle_max_)
//  {
//    angle_ = angle_min_;
//    radius_ += radius_step_;
//    if (radius_ >= radius_max_)
//    {
//      radius_ = radius_min_;
//      ++index_;
//    }
//  }
//
//  update_view_params();
//
//  return *this;
//}

/**
 * @return the distance from the current camera position to the object origin
 */
float
RendererIterator::D_obj()
{
  if (view_params_never_updated)
  {
	update_view_params();
	view_params_never_updated = false;
  }

  return radius_;
}

/**
 * @param T the translation vector
 * @param up the up vector of the view point
 */

void
RendererIterator::view_params(cv::Vec3d &T, cv::Vec3d &up)
{
  if (view_params_never_updated)
  {
	update_view_params();
	view_params_never_updated = false;
  }

  T = T_;
  up = up_;
}

RendererIterator &
RendererIterator::operator++()
{
  update_view_params();

  ++index_;

  return *this;
}
