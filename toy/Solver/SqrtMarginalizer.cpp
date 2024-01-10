#include <algorithm>
#include "config.h"
#include "ToyAssert.h"
#include "Frame.h"
#include "SqrtMarginalizationCost.h"
#include "SqrtMarginalizer.h"
#include "DebugUtil.h"


namespace toy {
void SqrtMarginalizer::setFrames(std::vector<db::Frame::Ptr>& frames) {
  auto size = mJ.cols() / db::Frame::PARAMETER_SIZE;
  mFrames.resize(size);
  std::copy(frames.begin(), frames.begin() + size, mFrames.begin());
}

std::shared_ptr<SqrtMarginalizationCost> SqrtMarginalizer::createMarginCost() {
  return std::make_shared<SqrtMarginalizationCost>(mFrames, mJ, mRes);
}

void SqrtMarginalizer::marginalize(Eigen::VectorXi& indices,
                                   Eigen::MatrixXd& J,
                                   Eigen::VectorXd& Res) {
  using PermutationWrapper = Eigen::PermutationWrapper<
    Eigen::Matrix<int, Eigen::Dynamic, 1>>;

  const PermutationWrapper permutation(indices);
  J.applyOnTheRight(permutation);

  size_t margRank       = 0;
  size_t validBlockRows = 0;
  decomposeWithQR(J, Res, db::Frame::PARAMETER_SIZE, margRank, validBlockRows);

  size_t& rows = validBlockRows;
  size_t  cols = indices.rows() - db::Frame::PARAMETER_SIZE;

  mJ.resize(rows, cols);
  mRes.resize(rows);

  mJ   = J.block(margRank, db::Frame::PARAMETER_SIZE, rows, cols);
  mRes = Res.segment(margRank, rows);

  //debug::drawSparseMatrix("after QR", mJ , 1);

  //ToyLogD("final QR {}", ToyLogger::eigenMat(mJ, 2));
  //ToyLogD("=============================================================");
}

void SqrtMarginalizer::decomposeWithQR(Eigen::MatrixXd& J,
                                       Eigen::VectorXd& Res,
                                       const size_t&    marginBlockSize,
                                       size_t&          marginRank,
                                       size_t&          validBlockRows) {
  //to check rank deficiency
  const Eigen::Index rows = J.rows();
  const Eigen::Index cols = J.cols();
  Eigen::VectorXd    vecBuffer(cols + 1);

  double* pBuffer = vecBuffer.data();

  Eigen::Index margRank  = 0;
  Eigen::Index totalRank = 0;

  double       beta, tau;
  const double betaThreshold = std::sqrt(std::numeric_limits<double>::epsilon());

  for (Eigen::Index k = 0; k < cols && totalRank < rows; ++k) {
    Eigen::Index remainingRows = rows - totalRank;
    Eigen::Index remainingCols = cols - k - 1;
    J.col(k).tail(remainingRows).makeHouseholderInPlace(tau, beta);

    if (std::abs(beta) > betaThreshold) {
      J.coeffRef(totalRank, k) = beta;

      J.bottomRightCorner(remainingRows, remainingCols)
        .applyHouseholderOnTheLeft(J.col(k).tail(remainingRows - 1),
                                   tau,
                                   pBuffer + k + 1);
      Res.tail(remainingRows)
        .applyHouseholderOnTheLeft(J.col(k).tail(remainingRows - 1), tau, pBuffer + cols);
      totalRank++;
    }
    else {
      J.coeffRef(totalRank, k) = 0;
    }
    //Overwrite householder vectors with 0
    J.col(k).tail(remainingRows - 1).setZero();

    //Save the rank of marginalize-out part
    if (k == marginBlockSize - 1) {
      marginRank = totalRank;
    }
  }
  validBlockRows = std::max(totalRank - marginRank, size_t{1});
}
}  //namespace toy