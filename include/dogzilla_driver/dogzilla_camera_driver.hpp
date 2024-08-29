#pragma once

#include <opencv2/opencv.hpp>

class DogzillaCameraDriver
{
public:
  explicit DogzillaCameraDriver(int video_id = 0, int width = 640, int height = 480);
  ~DogzillaCameraDriver();

  DogzillaCameraDriver(const DogzillaCameraDriver &) = delete;
  DogzillaCameraDriver &operator=(const DogzillaCameraDriver &) = delete;

  DogzillaCameraDriver(DogzillaCameraDriver &&) noexcept = default;
  DogzillaCameraDriver &operator=(DogzillaCameraDriver &&) noexcept = default;

private:
  auto initCamera() -> void;
  auto configCamera() -> void;
  
  int video_id_;
  int width_;
  int height_;
  cv::VideoCapture video_;
};

DogzillaCameraDriver::DogzillaCameraDriver(int video_id, int width, int height)
  : video_id_{ video_id }, width_{ width }, height_{ height }
{
  initCamera();
}

DogzillaCameraDriver::~DogzillaCameraDriver()
{
  if (video_.isOpened()) { video_.release(); }
}

auto DogzillaCameraDriver::initCamera() -> void
{
  video_.open(video_id_);
  if (!video_.isOpened()) {
    video_id_ = (video_id_ + 1) % 2;
    video_.open(video_id_);
    if (!video_.isOpened()) { std::cerr << "Error initializing camera."; }
  };
}

auto DogzillaCameraDriver::configCamera() -> void {}
