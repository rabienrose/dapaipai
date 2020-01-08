 

#ifndef __OPENCV_STITCHING_PRECOMP_H__
#define __OPENCV_STITCHING_PRECOMP_H__

#include "opencv2/opencv_modules.hpp"

#include <vector>
#include <algorithm>
#include <utility>
#include <set>
#include <functional>
#include <sstream>
#include <iostream>
#include <cmath>
#include "opencv2/core.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/utility.hpp"
#include "chamo/stitching/panod/autocalib.hpp"
#include "chamo/stitching/panod/blenders.hpp"
#include "chamo/stitching/panod/timelapsers.hpp"
#include "chamo/stitching/panod/camera.hpp"
#include "chamo/stitching/panod/exposure_compensate.hpp"
#include "chamo/stitching/panod/matchers.hpp"
#include "chamo/stitching/panod/motion_estimators.hpp"
#include "chamo/stitching/panod/seam_finders.hpp"
#include "chamo/stitching/panod/util.hpp"
#include "chamo/stitching/panod/warpers.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

#endif
