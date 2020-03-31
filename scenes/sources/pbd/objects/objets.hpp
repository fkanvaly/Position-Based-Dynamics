#pragma once

#include "scenes/base/base.hpp"

#include "vcl/vcl.hpp"

#include "rigid_body.hpp"
#include "fix_rigid_body.hpp"
#include "cloth.hpp"
#include "fluid.hpp"
#include "granular.hpp"

struct boundary
{
    vcl::vec3 pts;
    vcl::vec3 normal;
    float mu_k;
    float mu_s;

    boundary(vcl::vec3 p, vcl::vec3 n, float ms, float mk)
        : pts(p), normal(n), mu_k(mk), mu_s(ms)
    {};
};
