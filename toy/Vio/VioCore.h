namespace toy {
namespace db {
ImagePyramid;
}  //namespace db
class FrameTracker;
class LocalTracker;
class VioCore {
public:
  VioCore();
  ~VioCore();
  void insert(db::ImagePyramid* imagePyramids);
  void prepare();

  void processSync();

private:
  FrameTracker* mFrameTracker;
  LocalTracker* mLocalTracker;
};
}  //namespace toy