#include "TerrainMap.hpp"

#include <limits>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>

#include <drc_utils/BotWrapper.hpp>

#include <maps/BotWrapper.hpp>
#include <maps/ViewClient.hpp>
#include <maps/DepthImage.hpp>
#include <maps/DepthImageView.hpp>

#include "FillMethods.hpp"

using namespace terrainmap;

struct TerrainMap::Helper {

  struct Listener : public maps::ViewClient::Listener {
    Helper* mHelper;
    Listener(Helper* iHelper) : mHelper(iHelper) {}
    void notifyCatalog(const bool iChanged) {}
    void notifyData(const int64_t iViewId) {
      if (!mHelper->mListening) return;
      if (iViewId != mHelper->mViewId) return;
      auto view = std::dynamic_pointer_cast<maps::DepthImageView>
        (mHelper->mViewClient->getView(mHelper->mViewId));
      if (view != NULL) {
        auto type = maps::DepthImage::TypeDisparity;
        mHelper->mPreviousDepths = view->getDepthImage()->getData(type);
      }
      mHelper->mLatestView = view;
      mHelper->computeNewDepths();
    }
  };

  int64_t mViewId;
  std::string mMapChannel;
  Eigen::Vector4d mFillPlane;
  bool mUseFootPose;
  bool mOverrideHeights;
  bool mShouldFillMissing;
  NormalMethod mNormalMethod;
  double mNormalRadius;
  std::shared_ptr<drc::BotWrapper> mBotWrapper;

  std::shared_ptr<maps::ViewClient> mViewClient;
  std::shared_ptr<Listener> mListener;
  std::shared_ptr<maps::DepthImageView> mLatestView;
  Eigen::Vector4d mLatestFootPlane;
  std::vector<float> mPreviousDepths;
  bool mListening;

  Helper(const std::shared_ptr<drc::BotWrapper>& iBotWrapper) {
    mBotWrapper = iBotWrapper;
    mUseFootPose = false;
    mOverrideHeights = false;
    mNormalMethod = NormalMethodTriangle;
    mShouldFillMissing = true;
    mNormalRadius = 0;
    mListening = false;
    mFillPlane << 0,0,1,0;
    mLatestFootPlane << 0,0,1,0;
    mViewClient.reset(new maps::ViewClient());
    std::shared_ptr<maps::BotWrapper> wrapper
      (new maps::BotWrapper(mBotWrapper->getLcm(), mBotWrapper->getBotParam(),
                            mBotWrapper->getBotFrames()));
    mViewClient->setBotWrapper(wrapper);
    mBotWrapper->getLcm()->subscribe("POSE_GROUND", &Helper::onGround, this);
    mListener.reset(new Listener(this));
  }

  // TODO: need this? could also use bot-frames from ground to local
  // need to make sure that info is synchronized even when listening paused
  void onGround(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const bot_core::pose_t* iMessage) {
    if (!mListening) return;
    Eigen::Quaterniond q(iMessage->orientation[0], iMessage->orientation[1],
                         iMessage->orientation[2], iMessage->orientation[3]);
    Eigen::Vector3d pos(iMessage->pos[0], iMessage->pos[1], iMessage->pos[2]);
    mLatestFootPlane.head<3>() = q.matrix().col(2);
    mLatestFootPlane[3] = -mLatestFootPlane.head<3>().dot(pos);
  }

  Eigen::Vector4d getLatestFootPlane() const {
    // TODO: below might be slow if called too frequently
    if (false) {
      Eigen::Isometry3d groundPose;
      mBotWrapper->getTransform("ground","local",groundPose);
      Eigen::Vector4d plane;
      plane.head<3>() = groundPose.rotation().col(2);
      plane[3] = -plane.head<3>().dot(groundPose.translation());
      return plane;
    }

    return mLatestFootPlane;
  }

  void computeNewDepths() {
    if (mShouldFillMissing) {
      auto view = mLatestView;
      auto plane = mUseFootPose ? getLatestFootPlane() : mFillPlane;
      if (mOverrideHeights) {
        FillMethods::fillEntireView(view, plane);
      }
      else {
        // fill 0.5m square area underneath robot
        // TODO: could use timestamp of latest view
        Eigen::Isometry3d groundToLocal;
        mBotWrapper->getTransform("ground","local",groundToLocal);
        Eigen::Vector3d pos = groundToLocal.translation();
        FillMethods::fillBox(view, pos, 0.5, plane);

        // fill holes in environment using neighbor values
        FillMethods::fillHolesIterative(view);
      }
    }
  }
 
};

TerrainMap::
TerrainMap(const std::shared_ptr<drc::BotWrapper>& iBotWrapper) {
  mHelper.reset(new Helper(iBotWrapper));
}

TerrainMap::
~TerrainMap() {
}

void TerrainMap::
setInfo(const int64_t iViewId, const std::string& iMapChannel) {
  mHelper->mViewId = iViewId;
  mHelper->mMapChannel = iMapChannel;
}

int64_t TerrainMap::
getViewId() const {
  return mHelper->mViewId;
}

std::string TerrainMap::
getMapChannel() const {
  return mHelper->mMapChannel;
}

void TerrainMap::
setFillPlane(const Eigen::Vector4d& iPlane) {
  mHelper->mFillPlane = iPlane;
  mHelper->mFillPlane /= iPlane.head<3>().norm();
}

void TerrainMap::
useFootPose(const bool iVal) {
  mHelper->mUseFootPose = iVal;
}

void TerrainMap::
overrideHeights(const bool iVal) {
  mHelper->mOverrideHeights = iVal;
}

void TerrainMap::
setNormalMethod(const NormalMethod iMethod) {
  mHelper->mNormalMethod = iMethod;
}

void TerrainMap::
setNormalRadius(const double iRadius) {
  mHelper->mNormalRadius = iRadius;
}

void TerrainMap::
shouldFillMissing(const bool iVal) {
  mHelper->mShouldFillMissing = iVal;
}

double TerrainMap::
getNormalRadius() const {
  return mHelper->mNormalRadius;
}

bool TerrainMap::
startListening() {
  if (mHelper->mListening) return false;
  mHelper->mViewClient->start();
  mHelper->mListening = true;
  return true;
}

bool TerrainMap::
stopListening() {
  if (!mHelper->mListening) return false;
  mHelper->mListening = false;
  mHelper->mViewClient->stop();
  return true;
}


bool TerrainMap::
sendRequest(const Eigen::Vector3d& iBoxMin, const Eigen::Vector3d& iBoxMax,
            const double iResolutionMeters, const double iTimeWindowSeconds,
            const double iFrequencyHz) {

  maps::ViewBase::Spec spec;
  spec.mResolution = (float)iResolutionMeters;
  spec.mWidth = int((iBoxMax[0] - iBoxMin[0]) / spec.mResolution);
  spec.mHeight = int((iBoxMax[1] - iBoxMin[1]) / spec.mResolution);
  spec.mTimeMin = -iTimeWindowSeconds*1e6;
  spec.mClipPlanes.push_back(Eigen::Vector4f( 1, 0, 0, -iBoxMin[0]));
  spec.mClipPlanes.push_back(Eigen::Vector4f(-1, 0, 0,  iBoxMax[0]));
  spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 1, 0, -iBoxMin[1]));
  spec.mClipPlanes.push_back(Eigen::Vector4f( 0,-1, 0,  iBoxMax[1]));
  spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 0, 1, -iBoxMin[2]));
  spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 0,-1,  iBoxMax[2]));

  spec.mMapId = 1;
  spec.mViewId = getViewId();
  spec.mType = maps::ViewBase::TypeDepthImage;
  spec.mChannel = getMapChannel();
  spec.mFrequency = iFrequencyHz;
  spec.mQuantizationMax = 0;
  spec.mTimeMax = 0;
  spec.mTimeMode = maps::ViewBase::TimeModeRelative;
  spec.mRelativeLocation = true;
  spec.mActive = true;
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.translation() = Eigen::Vector3f(0,0,10);
  pose.linear() << 1,0,0, 0,-1,0, 0,0,-1;
  Eigen::Affine3f calib = Eigen::Affine3f::Identity();
  calib(0,0) = 1/spec.mResolution;
  calib(1,1) = 1/spec.mResolution;
  calib(0,3) = -iBoxMin[0]*calib(0,0);
  calib(1,3) = -iBoxMin[1]*calib(1,1);
  Eigen::Projective3f projector = calib*pose.inverse();
  spec.mTransform = projector;

  // send request
  mHelper->mViewClient->request(spec);

  return true;
}

std::shared_ptr<TerrainMap::TerrainData> TerrainMap::
getData() const {
  auto view = mHelper->mLatestView;
  if (view == NULL) return NULL;
  std::shared_ptr<TerrainData> data(new TerrainData());
  auto type = maps::DepthImage::TypeDepth;
  auto image = view->getDepthImage();
  data->mHeights = image->getData(type);
  data->mWidth = image->getWidth();
  data->mHeight = image->getHeight();
  data->mTransform = view->getTransform().cast<double>();
  return data;
}

template <typename T>
bool TerrainMap::
getHeightAndNormal(const T iX, const T iY,
                   T& oHeight, Eigen::Matrix<T,3,1>& oNormal) const {
  bool usePlaneForHeight = mHelper->mOverrideHeights;
  bool usePlaneForNormal = (mHelper->mNormalMethod == NormalMethodOverride);
  if (!(usePlaneForHeight && usePlaneForNormal)) {
    Eigen::Vector3f pt, normal;
    const auto& view = mHelper->mLatestView;
    auto normalMethod = maps::DepthImageView::NormalMethodTriangle;
    switch (mHelper->mNormalMethod) {
    case NormalMethodOverride:
      normalMethod = maps::DepthImageView::NormalMethodZ; break;
    case NormalMethodTriangle:
      normalMethod = maps::DepthImageView::NormalMethodTriangle; break;
    case NormalMethodLeastSquares:
      normalMethod = maps::DepthImageView::NormalMethodLeastSquares; break;
    case NormalMethodRobustKernel:
      normalMethod = maps::DepthImageView::NormalMethodRobustKernel; break;
    case NormalMethodSampleConsensus:
      normalMethod = maps::DepthImageView::NormalMethodSampleConsensus; break;
    default: break;
    }
    view->setNormalMethod(normalMethod);
    if ((view == NULL) ||
        !view->getClosest(Eigen::Vector3f(iX, iY, 0), pt, normal)) {
      usePlaneForHeight = usePlaneForNormal = true;
    }
    oHeight = pt[2];
    oNormal = normal.cast<T>();
  }

  const typename Eigen::Matrix<T,4,1>& plane =
    (mHelper->mUseFootPose ? mHelper->getLatestFootPlane() :
     mHelper->mFillPlane).cast<T>();
  if (usePlaneForHeight) {
    oHeight -(plane[0]*iX + plane[1]*iY + plane[3])/plane[2];
  }
  if (usePlaneForNormal) {
    oNormal << plane[0],plane[1],plane[2];
  }
  return true;
}

// explicit instantiations
template bool TerrainMap::
getHeightAndNormal(const float iX, const float iY,
                   float& oHeight, Eigen::Vector3f& oNormal) const;
template bool TerrainMap::
getHeightAndNormal(const double iX, const double iY,
                   double& oHeight, Eigen::Vector3d& oNormal) const;
