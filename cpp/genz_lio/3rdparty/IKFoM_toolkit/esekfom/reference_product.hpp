// GenZ-LIO local extension; see 3rdparty/PATCHES.md.
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <vector>

namespace esekfom {
namespace detail {

// Compatibility reduction order for the validated Eigen 3.4 SSE2 reference.
// The 760-element bound and balanced remainder preserve that reference's
// observation accumulation order. They are a versioned numerical policy, NOT
// a claim about the executing CPU's cache. No global Eigen state is changed.
// SIMD/compiler differences across other builds remain a separate concern.
inline Eigen::Index referenceObservationBlock(Eigen::Index depth) {
    constexpr Eigen::Index bound = 760;
    constexpr Eigen::Index peel = 8;
    if (depth <= bound) return depth;
    return depth % bound == 0 ? bound
        : bound - peel * ((bound - 1 - depth % bound) / (peel * (depth / bound + 1)));
}

template <typename Scalar>
class ObservationBlocking : public Eigen::internal::level3_blocking<Scalar, Scalar> {
    std::vector<Scalar, Eigen::aligned_allocator<Scalar>> lhs_, rhs_;
public:
    ObservationBlocking(Eigen::Index rows, Eigen::Index cols, Eigen::Index depth)
        : lhs_(rows * referenceObservationBlock(depth)),
          rhs_(cols * referenceObservationBlock(depth)) {
        this->m_mc = rows;
        this->m_nc = cols;
        this->m_kc = referenceObservationBlock(depth);
        this->m_blockA = lhs_.data();
        this->m_blockB = rhs_.data();
    }
};

// Only the two tall observation contractions in the diagonal update use this
// policy. Both operands must be plain, contiguous column-major real matrices.
// All other Eigen products retain their normal implementation.
template <typename Left, typename Right>
auto referenceObservationProduct(const Eigen::MatrixBase<Left>& left,
                                 const Eigen::MatrixBase<Right>& right) {
    using Scalar = typename Left::Scalar;
    static_assert(!Left::IsRowMajor && !Right::IsRowMajor, "column-major inputs required");
    static_assert(Right::ColsAtCompileTime == 12, "observation Jacobian must have 12 columns");
    Eigen::Matrix<Scalar, Left::RowsAtCompileTime, 12> result(left.rows(), 12);
    eigen_assert(left.cols() == right.rows());
    if (left.cols() < 48) {
        result.noalias() = left * right;
        return result;
    }
    result.setZero();
    ObservationBlocking<Scalar> blocking(left.rows(), right.cols(), left.cols());
    Eigen::internal::general_matrix_matrix_product<Eigen::Index,
        Scalar, Eigen::ColMajor, false, Scalar, Eigen::ColMajor, false,
        Eigen::ColMajor, 1>::run(left.rows(), right.cols(), left.cols(),
        left.derived().data(), left.outerStride(), right.derived().data(), right.outerStride(),
        result.data(), 1, result.outerStride(), Scalar(1), blocking, nullptr);
    return result;
}

}  // namespace detail
}  // namespace esekfom
