#pragma once

namespace toy {
/**
 * @brief
 * MAIN  : main camera's image
 * SUB   : In a stereo camera setup, SUB refers to the image from the camera
 * positioned next to the main camera. DEPTH : Depth image
 */
enum class ImageType { MAIN = 0, SUB = 1, DEPTH = 2 };
enum class ImageFormat { GRAY = 0, RGB888 = 1 };

};  //namespace toy