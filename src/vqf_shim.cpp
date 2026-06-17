/* vqf_shim.cpp - C-callable wrapper around VQF (dlaidig/vqf, MIT). See vqf_shim.h. */
#include "vqf_shim.h"
#include "vqf/vqf.hpp"

extern "C" {

vqf_handle *vqf_create(double dt){ return reinterpret_cast<vqf_handle*>(new VQF(dt)); }
void vqf_destroy(vqf_handle *h){ delete reinterpret_cast<VQF*>(h); }

void vqf_update6(vqf_handle *h, const double g[3], const double a[3]){
    reinterpret_cast<VQF*>(h)->update(g, a);
}
void vqf_update9(vqf_handle *h, const double g[3], const double a[3], const double m[3]){
    reinterpret_cast<VQF*>(h)->update(g, a, m);
}
void vqf_quat6(vqf_handle *h, double o[4]){ reinterpret_cast<VQF*>(h)->getQuat6D(o); }
void vqf_quat9(vqf_handle *h, double o[4]){ reinterpret_cast<VQF*>(h)->getQuat9D(o); }

void vqf_get_bias(vqf_handle *h, double o[3]){ reinterpret_cast<VQF*>(h)->getBiasEstimate(o); }
int  vqf_rest_detected(vqf_handle *h){ return reinterpret_cast<VQF*>(h)->getRestDetected() ? 1 : 0; }
int  vqf_mag_dist_detected(vqf_handle *h){ return reinterpret_cast<VQF*>(h)->getMagDistDetected() ? 1 : 0; }

}
