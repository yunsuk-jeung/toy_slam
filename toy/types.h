#pragma once

namespace toy {
/**
 * @brief
 * MAIN  : main camera's image
 * SUB   : In a stereo camera setup, SUB refers to the image from the camera positioned
 * next to the main camera. DEPTH : Depth image
 */
enum class ImageType { MAIN, SUB, DEPTH };
enum class ImageFormat { GRAAY, RGB888 };

};  //namespace toy