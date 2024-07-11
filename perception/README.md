

# alignment error
https://eigen.tuxfamily.org/dox/group__DenseMatrixManipulation__Alignement.html
https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html


https://stackoverflow.com/questions/36211864/how-can-i-apply-attribute-aligned32-to-an-int



use `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` and `Eigen::aligned_allocator` in class
use `__attribute__ ((aligned (32))) `

```c++
    auto  __attribute__ ((aligned (32)))  start_relative_pose = pallet_candidates[start_id].pallet_pose_inv *candidate.pallet_pose;

```

```c++
    struct PalletCluster{
        std::vector<PalletCandidate,Eigen::aligned_allocator<PalletCandidate>> candidates;

         Eigen::Transform<double,3,Eigen::Isometry> est_pose;
         Eigen::Transform<double,3,Eigen::Isometry> est_pose_inv;

        i32_t valid_status;

        u32_t top_row_high;
        u32_t top_row_low;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
```


# math

https://github.com/artivis/manif

