/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <boost/bind/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "rviz_rotatable_image_plugin/rotatable_image_display.h"

namespace rviz
{
ImageDisplay2::ImageDisplay2() : ImageDisplayBase(), texture_()
{
  normalize_property_ = new BoolProperty(
      "Normalize Range", true,
      "If set to true, will try to estimate the range of possible values from the received images.",
      this, SLOT(updateNormalizeOptions()));

  min_property_ = new FloatProperty("Min Value", 0.0, "Value which will be displayed as black.", this,
                                    SLOT(updateNormalizeOptions()));

  max_property_ = new FloatProperty("Max Value", 1.0, "Value which will be displayed as white.", this,
                                    SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ =
      new IntProperty("Median window", 5, "Window size for median filter used for computin min/max.",
                      this, SLOT(updateNormalizeOptions()));

  crop_image_property_ = new BoolProperty("Cropped Rotation", true, 
                                    "If set to true, will not resize rotated image.", this,
                                    SLOT(updateRotationOptions()));

  rotation_angle_property_ = new FloatProperty("Rotation Degrees", 0.0, "Angle (in degrees) to which to rotate the image.", this,
                                    SLOT(updateRotationOptions()));

  got_float_image_ = false;
}

void ImageDisplay2::onInitialize()
{
  ImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "ImageDisplay2" << count++;
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
  }

  img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "ImageDisplay2Object" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(false);
    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);

    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    setMaterial(*screen_rect_, material_);
    img_scene_node_->attachObject(screen_rect_);
  }

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(false);

  render_panel_->resize(640, 480);
  render_panel_->initialize(img_scene_manager_, context_);

  setAssociatedWidget(render_panel_);

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance(0.01f);

  updateNormalizeOptions();
}

ImageDisplay2::~ImageDisplay2()
{
  if (initialized())
  {
    delete render_panel_;
    delete screen_rect_;
    removeAndDestroyChildNode(img_scene_node_->getParentSceneNode(), img_scene_node_);
  }
}

void ImageDisplay2::onEnable()
{
  ImageDisplayBase::subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void ImageDisplay2::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);
  ImageDisplayBase::unsubscribe();
  reset();
}

void ImageDisplay2::updateNormalizeOptions()
{
  if (got_float_image_)
  {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_.setNormalizeFloatImage(normalize, min_property_->getFloat(), max_property_->getFloat());
    texture_.setMedianFrames(median_buffer_size_property_->getInt());
  }
  else
  {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void ImageDisplay2::updateRotationOptions()
{
  should_crop_image_ = crop_image_property_->getBool();
  angle_to_rotate_ = rotation_angle_property_->getFloat();
}


void ImageDisplay2::update(float wall_dt, float ros_dt)
{
  Q_UNUSED(wall_dt)
  Q_UNUSED(ros_dt)
  try
  {
    texture_.update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_.getWidth();
    float img_height = texture_.getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0)
    {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect)
      {
        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect / img_aspect, 1.0f,
                                 -1.0f * win_aspect / img_aspect, false);
      }
      else
      {
        screen_rect_->setCorners(-1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect,
                                 -1.0f, false);
      }
    }

    render_panel_->getRenderWindow()->update();
  }
  catch (UnsupportedImageEncoding& e)
  {
    setStatus(StatusProperty::Error, "Image", e.what());
  }
}

void ImageDisplay2::reset()
{
  ImageDisplayBase::reset();
  texture_.clear();
  render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

/* This is called by incomingMessage(). */
void ImageDisplay2::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
  bool got_float_image = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                         msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                         msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
                         msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_)
  {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }

  if (angle_to_rotate_)
  {
    cv_bridge::CvImageConstPtr msg_cv_ptr;
    try
    {
      if (sensor_msgs::image_encodings::isColor(msg->encoding))
      {
        msg_cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      }
      else
      {
        msg_cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat rotated_image;
    sensor_msgs::ImagePtr rotated_image_msg;

    cv::Point2f center_of_image(0.5*msg_cv_ptr->image.cols, 0.5*msg_cv_ptr->image.rows);
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center_of_image, angle_to_rotate_, 1.0);

    if (should_crop_image_)
    {
      cv::warpAffine(msg_cv_ptr->image, rotated_image, rotation_matrix, msg_cv_ptr->image.size());
    }
    else
    {
      // determine bounding rectangle, center not relevant
      cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), msg_cv_ptr->image.size(), angle_to_rotate_).boundingRect2f();
      // adjust transformation matrix
      rotation_matrix.at<double>(0,2) += 0.5*bbox.width - 0.5*msg_cv_ptr->image.cols;
      rotation_matrix.at<double>(1,2) += 0.5*bbox.height - 0.5*msg_cv_ptr->image.rows;

      cv::warpAffine(msg_cv_ptr->image, rotated_image, rotation_matrix, bbox.size());
    }

    // convert to sensor_msgs::ImagePtr
    if (sensor_msgs::image_encodings::isColor(msg->encoding))
    {
      rotated_image_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, rotated_image).toImageMsg();
    }
    else
    {
      rotated_image_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, rotated_image).toImageMsg();
    }

    texture_.addMessage(rotated_image_msg);
  }
  else
  {
    texture_.addMessage(msg);
  }
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::ImageDisplay2, rviz::Display)

