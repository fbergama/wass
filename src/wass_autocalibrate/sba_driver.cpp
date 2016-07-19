/*************************************************************************

WASS - Wave Acquisition Stereo System
Copyright (C) 2016  Filippo Bergamasco

-------------------------------------------------------------------------

Most of the functions listed in this source file have been copied
and adapted from "eucsbademo.c" in the software package:

sba : A Generic Sparse Bundle Adjustment C/C++ Package Based on
the Levenberg-Marquardt Algorithm
(http://users.ics.forth.gr/~lourakis/sba/)

by Manolis Lourakis under GPL v2 license


*************************************************************************/


#include "sba_driver.h"
#include <sba.h>
#include <cmath>
#include "log.hpp"

#define SBA_MAX_REPROJ_ERROR    4.0 // max motion only reprojection error

#define BA_NONE                 -1
#define BA_MOTSTRUCT            0
#define BA_MOT                  1
#define BA_STRUCT               2
#define BA_MOT_MOTSTRUCT        3

#define FULLQUATSZ 4

/* unit quaternion from vector part */
#define _MK_QUAT_FRM_VEC(q, v){                                     \
    (q)[1]=(v)[0]; (q)[2]=(v)[1]; (q)[3]=(v)[2];                      \
    (q)[0]=sqrt(1.0 - (q)[1]*(q)[1] - (q)[2]*(q)[2]- (q)[3]*(q)[3]);  \
}

/*
 * fast multiplication of the two quaternions in q1 and q2 into p
 * this is the second of the two schemes derived in pg. 8 of
 * T. D. Howell, J.-C. Lafon, The complexity of the quaternion product, TR 75-245, Cornell Univ., June 1975.
 *
 * total additions increase from 12 to 27 (28), but total multiplications decrease from 16 to 9 (12)
 */
inline static void quatMultFast(double q1[FULLQUATSZ], double q2[FULLQUATSZ], double p[FULLQUATSZ])
{
    double t1, t2, t3, t4, t5, t6, t7, t8, t9;
    //double t10, t11, t12;

    t1=(q1[0]+q1[1])*(q2[0]+q2[1]);
    t2=(q1[3]-q1[2])*(q2[2]-q2[3]);
    t3=(q1[1]-q1[0])*(q2[2]+q2[3]);
    t4=(q1[2]+q1[3])*(q2[1]-q2[0]);
    t5=(q1[1]+q1[3])*(q2[1]+q2[2]);
    t6=(q1[1]-q1[3])*(q2[1]-q2[2]);
    t7=(q1[0]+q1[2])*(q2[0]-q2[3]);
    t8=(q1[0]-q1[2])*(q2[0]+q2[3]);

#if 0
    t9 =t5+t6;
    t10=t7+t8;
    t11=t5-t6;
    t12=t7-t8;

    p[0]= t2 + 0.5*(-t9+t10);
    p[1]= t1 - 0.5*(t9+t10);
    p[2]=-t3 + 0.5*(t11+t12);
    p[3]=-t4 + 0.5*(t11-t12);
#endif

    /* following fragment it equivalent to the one above */
    t9=0.5*(t5-t6+t7+t8);
    p[0]= t2 + t9-t5;
    p[1]= t1 - t9-t6;
    p[2]=-t3 + t9-t8;
    p[3]=-t4 + t9-t7;
}



/* pointers to additional data, used for computed image projections and their jacobians */
struct globs_{
    double *rot0params; /* initial rotation parameters, combined with a local rotation parameterization */
    double *intrcalib; /* the 5 intrinsic calibration parameters in the order [fu, u0, v0, ar, skew],
                        * where ar is the aspect ratio fv/fu.
                        * Used only when calibration is fixed for all cameras;
                        * otherwise, it is null and the intrinsic parameters are
                        * included in the set of motion parameters for each camera
                        */
    int nccalib; /* number of calibration parameters that must be kept constant.
                  * 0: all parameters are free
                  * 1: skew is fixed to its initial value, all other parameters vary (i.e. fu, u0, v0, ar)
                  * 2: skew and aspect ratio are fixed to their initial values, all other parameters vary (i.e. fu, u0, v0)
                  * 3: meaningless
                  * 4: skew, aspect ratio and principal point are fixed to their initial values, only the focal length varies (i.e. fu)
                  * 5: all intrinsics are kept fixed to their initial values
                  * >5: meaningless
                  * Used only when calibration varies among cameras
                  */

    int ncdist; /* number of distortion parameters in Bouguet's model that must be kept constant.
                 * 0: all parameters are free
                 * 1: 6th order radial distortion term (kc[4]) is fixed
                 * 2: 6th order radial distortion and one of the tangential distortion terms (kc[3]) are fixed
                 * 3: 6th order radial distortion and both tangential distortion terms (kc[3], kc[2]) are fixed [i.e., only 2nd & 4th order radial dist.]
                 * 4: 4th & 6th order radial distortion terms and both tangential distortion ones are fixed [i.e., only 2nd order radial dist.]
                 * 5: all distortion parameters are kept fixed to their initial values
                 * >5: meaningless
                 * Used only when calibration varies among cameras and distortion is to be estimated
                 */
    int cnp, pnp, mnp; /* dimensions */

    double *ptparams; /* needed only when bundle adjusting for camera parameters only */
    double *camparams; /* needed only when bundle adjusting for structure parameters only */
} globs;



/* convert a vector of camera parameters so that rotation is represented by
 * the vector part of the input quaternion. The function converts the
 * input quaternion into a unit one with a non-negative scalar part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
 */
void quat2vec(double *inp, int nin, double *outp, int nout)
{
    double mag, sg;
    int i;

    /* intrinsics & distortion */
    if(nin>7) // are they present?
        for(i=0; i<nin-7; ++i)
            outp[i]=inp[i];
    else
        i=0;

    /* rotation */
    /* normalize and ensure that the quaternion's scalar component is non-negative;
     * if not, negate the quaternion since two quaternions q and -q represent the
     * same rotation
     */
    mag=sqrt(inp[i]*inp[i] + inp[i+1]*inp[i+1] + inp[i+2]*inp[i+2] + inp[i+3]*inp[i+3]);
    sg=(inp[i]>=0.0)? 1.0 : -1.0;
    mag=sg/mag;
    outp[i]  =inp[i+1]*mag;
    outp[i+1]=inp[i+2]*mag;
    outp[i+2]=inp[i+3]*mag;
    i+=3;

    /* translation*/
    for( ; i<nout; ++i)
        outp[i]=inp[i+1];
}


/* convert a vector of camera parameters so that rotation is represented by
 * a full unit quaternion instead of its input 3-vector part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
 */
void vec2quat(double *inp, int nin, double *outp, int nout)
{
    double *v, q[FULLQUATSZ];
    int i;

    /* intrinsics & distortion */
    if(nin>7-1) // are they present?
        for(i=0; i<nin-(7-1); ++i)
            outp[i]=inp[i];
    else
        i=0;

    /* rotation */
    /* recover the quaternion from the vector */
    v=inp+i;
    _MK_QUAT_FRM_VEC(q, v);
    outp[i]  =q[0];
    outp[i+1]=q[1];
    outp[i+2]=q[2];
    outp[i+3]=q[3];
    i+=FULLQUATSZ;

    /* translation */
    for( ; i<nout; ++i)
        outp[i]=inp[i-1];
}


cv::Vec4d rotation_mat_to_quaternion( const cv::Matx44d& m /* uses the upper 3x3 matrix of m */) {

    const double& m00 = m(0,0);
    const double& m11 = m(1,1);
    const double& m22 = m(2,2);
    const double& m01 = m(0,1);
    const double& m02 = m(0,2);
    const double& m10 = m(1,0);
    const double& m12 = m(1,2);
    const double& m20 = m(2,0);
    const double& m21 = m(2,1);

    cv::Vec4d ret;

    double ww = 0.25 * (m00 + m11 + m22 + 1.);

    if (ww >= 0.) {
        if (ww >= 1e-30) {
            ret[0] = std::sqrt(ww);
            ww = 0.25/ret[0];
            ret[1] = (m21 - m12)*ww;
            ret[2] = (m02 - m20)*ww;
            ret[3] = (m10 - m01)*ww;
            return ret;
        }
    } else {
        ret[0] = 0.;
        ret[1] = 0.;
        ret[2] = 0.;
        ret[3] = 1.;
        return ret;
    }

    ret[0] = 0.;
    ww = -0.5*(m11 + m22);
    if (ww >= 0.) {
        if (ww >= 1e-30) {
            ret[1] = std::sqrt(ww);
            ww = 0.5/ret[1];
            ret[2] = m10*ww;
            ret[3] = m20*ww;
            return ret;
        }
    } else {
        ret[1] = 0.;
        ret[2] = 0.;
        ret[3] = 1.;
        return ret;
    }

    ret[1] = 0.;
    ww = 0.5*(1. - m22);
    if (ww >= 1e-30) {
        ret[2] = std::sqrt(ww);
        ret[3] = m21/(2.*ret[2]);
        return ret;
    }

    ret[2] = 0.;
    ret[3] = 1.;

    return ret;
}

cv::Matx33d quaternion_to_rot_matrix( const cv::Vec4d q )
{
    cv::Matx33d m;

    m(0,0) = 1 - 2*q[2]*q[2] - 2*q[3]*q[3];
    m(0,1) = 2*q[1]*q[2] - 2*q[3]*q[0];
    m(0,2) = 2*q[1]*q[3] + 2*q[2]*q[0];

    m(1,0) = 2*q[1]*q[2] + 2*q[3]*q[0];
    m(1,1) = 1 - 2*q[1]*q[1] - 2*q[3]*q[3];
    m(1,2) = 2*q[2]*q[3] - 2*q[1]*q[0];

    m(2,0) = 2*q[1]*q[3] - 2*q[2]*q[0];
    m(2,1) = 2*q[2]*q[3] + 2*q[1]*q[0];
    m(2,2) = 1 - 2*q[1]*q[1] - 2*q[2]*q[2];

    return m;
}


void calcImgProjFullR(double a[5],double qr0[4],double t[3],double M[3],
        double n[2])
{
    double t1;
    double t11;
    double t13;
    double t17;
    double t2;
    double t22;
    double t27;
    double t3;
    double t38;
    double t46;
    double t49;
    double t5;
    double t6;
    double t8;
    double t9;
    {
        t1 = a[0];
        t2 = qr0[1];
        t3 = M[0];
        t5 = qr0[2];
        t6 = M[1];
        t8 = qr0[3];
        t9 = M[2];
        t11 = -t3*t2-t5*t6-t8*t9;
        t13 = qr0[0];
        t17 = t13*t3+t5*t9-t8*t6;
        t22 = t6*t13+t8*t3-t9*t2;
        t27 = t13*t9+t6*t2-t5*t3;
        t38 = -t5*t11+t13*t22-t27*t2+t8*t17+t[1];
        t46 = -t11*t8+t13*t27-t5*t17+t2*t22+t[2];
        t49 = 1/t46;
        n[0] = (t1*(-t2*t11+t13*t17-t22*t8+t5*t27+t[0])+a[4]*t38+a[1]*t46)*t49;
        n[1] = (t1*a[3]*t38+a[2]*t46)*t49;
        return;
    }
}

void calcImgProjJacRTS(double a[5],double qr0[4],double v[3],double t[3],
        double M[3],double jacmRT[2][6],double jacmS[2][3])
{
    double t1;
    double t10;
    double t107;
    double t109;
    double t11;
    double t118;
    double t12;
    double t126;
    double t127;
    double t14;
    double t141;
    double t145;
    double t146;
    double t147;
    double t15;
    double t150;
    double t152;
    double t159;
    double t16;
    double t162;
    double t165;
    double t168;
    double t170;
    double t172;
    double t175;
    double t18;
    double t180;
    double t185;
    double t187;
    double t19;
    double t192;
    double t194;
    double t2;
    double t206;
    double t21;
    double t216;
    double t22;
    double t227;
    double t23;
    double t230;
    double t233;
    double t235;
    double t237;
    double t240;
    double t245;
    double t25;
    double t250;
    double t252;
    double t257;
    double t259;
    double t27;
    double t271;
    double t28;
    double t281;
    double t293;
    double t294;
    double t296;
    double t299;
    double t3;
    double t30;
    double t302;
    double t303;
    double t305;
    double t306;
    double t309;
    double t324;
    double t325;
    double t327;
    double t330;
    double t331;
    double t347;
    double t35;
    double t350;
    double t37;
    double t4;
    double t43;
    double t49;
    double t5;
    double t51;
    double t52;
    double t54;
    double t56;
    double t6;
    double t61;
    double t65;
    double t7;
    double t70;
    double t75;
    double t76;
    double t81;
    double t82;
    double t87;
    double t88;
    double t9;
    double t93;
    double t94;
    double t98;
    {
        t1 = a[0];
        t2 = v[0];
        t3 = t2*t2;
        t4 = v[1];
        t5 = t4*t4;
        t6 = v[2];
        t7 = t6*t6;
        t9 = sqrt(1.0-t3-t5-t7);
        t10 = 1/t9;
        t11 = qr0[1];
        t12 = t11*t10;
        t14 = qr0[0];
        t15 = -t12*t2+t14;
        t16 = M[0];
        t18 = qr0[2];
        t19 = t18*t10;
        t21 = qr0[3];
        t22 = -t19*t2-t21;
        t23 = M[1];
        t25 = t10*t21;
        t27 = -t25*t2+t18;
        t28 = M[2];
        t30 = -t15*t16-t22*t23-t27*t28;
        t35 = -t9*t11-t2*t14-t4*t21+t6*t18;
        t37 = -t35;
        t43 = t9*t18+t4*t14+t6*t11-t2*t21;
        t49 = t9*t21+t6*t14+t2*t18-t11*t4;
        t51 = -t37*t16-t43*t23-t49*t28;
        t52 = -t15;
        t54 = t10*t14;
        t56 = -t54*t2-t11;
        t61 = t9*t14-t2*t11-t4*t18-t6*t21;
        t65 = t61*t16+t43*t28-t23*t49;
        t70 = t56*t16+t22*t28-t23*t27;
        t75 = t56*t23+t27*t16-t28*t15;
        t76 = -t49;
        t81 = t61*t23+t49*t16-t37*t28;
        t82 = -t27;
        t87 = t56*t28+t23*t15-t22*t16;
        t88 = -t43;
        t93 = t61*t28+t37*t23-t43*t16;
        t94 = -t22;
        t98 = a[4];
        t107 = t30*t88+t94*t51+t56*t81+t61*t75+t87*t35+t93*t52-t70*t76-t82*t65;
        t109 = a[1];
        t118 = t30*t76+t82*t51+t56*t93+t61*t87+t70*t88+t65*t94-t35*t75-t81*t52;
        t126 = t76*t51+t61*t93+t65*t88-t81*t35+t[2];
        t127 = 1/t126;
        t141 = t51*t88+t61*t81+t93*t35-t65*t76+t[1];
        t145 = t126*t126;
        t146 = 1/t145;
        t147 = (t1*(t35*t51+t61*t65+t81*t76-t93*t88+t[0])+t98*t141+t126*t109)*t146;
        jacmRT[0][0] = (t1*(t30*t35+t52*t51+t56*t65+t61*t70+t76*t75+t81*t82-t88*t87
                    -t93*t94)+t98*t107+t109*t118)*t127-t118*t147;
        t150 = t1*a[3];
        t152 = a[2];
        t159 = (t150*t141+t126*t152)*t146;
        jacmRT[1][0] = (t107*t150+t152*t118)*t127-t159*t118;
        t162 = -t12*t4+t21;
        t165 = -t19*t4+t14;
        t168 = -t25*t4-t11;
        t170 = -t162*t16-t165*t23-t168*t28;
        t172 = -t162;
        t175 = -t54*t4-t18;
        t180 = t175*t16+t165*t28-t168*t23;
        t185 = t175*t23+t168*t16-t162*t28;
        t187 = -t168;
        t192 = t175*t28+t162*t23-t165*t16;
        t194 = -t165;
        t206 = t170*t88+t51*t194+t175*t81+t61*t185+t192*t35+t93*t172-t76*t180-t65*
            t187;
        t216 = t170*t76+t51*t187+t93*t175+t61*t192+t180*t88+t65*t194-t185*t35-t81*
            t172;
        jacmRT[0][1] = (t1*(t170*t35+t172*t51+t175*t65+t180*t61+t185*t76+t81*t187-
                    t192*t88-t93*t194)+t98*t206+t109*t216)*t127-t147*t216;
        jacmRT[1][1] = (t150*t206+t152*t216)*t127-t159*t216;
        t227 = -t12*t6-t18;
        t230 = -t19*t6+t11;
        t233 = -t25*t6+t14;
        t235 = -t227*t16-t23*t230-t233*t28;
        t237 = -t227;
        t240 = -t54*t6-t21;
        t245 = t240*t16+t230*t28-t233*t23;
        t250 = t23*t240+t233*t16-t227*t28;
        t252 = -t233;
        t257 = t240*t28+t227*t23-t230*t16;
        t259 = -t230;
        t271 = t235*t88+t51*t259+t81*t240+t61*t250+t257*t35+t93*t237-t245*t76-t65*
            t252;
        t281 = t235*t76+t51*t252+t240*t93+t61*t257+t245*t88+t259*t65-t250*t35-t81*
            t237;
        jacmRT[0][2] = (t1*(t235*t35+t237*t51+t240*t65+t61*t245+t250*t76+t81*t252-
                    t257*t88-t93*t259)+t271*t98+t281*t109)*t127-t147*t281;
        jacmRT[1][2] = (t150*t271+t281*t152)*t127-t159*t281;
        jacmRT[0][3] = t127*t1;
        jacmRT[1][3] = 0.0;
        jacmRT[0][4] = t98*t127;
        jacmRT[1][4] = t150*t127;
        jacmRT[0][5] = t109*t127-t147;
        jacmRT[1][5] = t152*t127-t159;
        t293 = t35*t35;
        t294 = t61*t61;
        t296 = t88*t88;
        t299 = t35*t88;
        t302 = t61*t76;
        t303 = 2.0*t299+t61*t49-t302;
        t305 = t35*t76;
        t306 = t61*t88;
        t309 = t305+2.0*t306-t49*t35;
        jacmS[0][0] = (t1*(t293+t294+t49*t76-t296)+t98*t303+t109*t309)*t127-t147*
            t309;
        jacmS[1][0] = (t150*t303+t152*t309)*t127-t159*t309;
        t324 = t76*t76;
        t325 = t296+t294+t35*t37-t324;
        t327 = t76*t88;
        t330 = t61*t35;
        t331 = 2.0*t327+t61*t37-t330;
        jacmS[0][1] = (t1*(t299+2.0*t302-t37*t88)+t98*t325+t109*t331)*t127-t147*
            t331;
        jacmS[1][1] = (t150*t325+t152*t331)*t127-t159*t331;
        t347 = t327+2.0*t330-t43*t76;
        t350 = t324+t294+t43*t88-t293;
        jacmS[0][2] = (t1*(2.0*t305+t61*t43-t306)+t98*t347+t350*t109)*t127-t147*
            t350;
        jacmS[1][2] = (t150*t347+t152*t350)*t127-t159*t350;
        return;
    }
}




/*** MEASUREMENT VECTOR AND JACOBIAN COMPUTATION FOR THE EXPERT DRIVERS ***/

/* FULL BUNDLE ADJUSTMENT, I.E. SIMULTANEOUS ESTIMATION OF CAMERA AND STRUCTURE PARAMETERS */

/* Given a parameter vector p made up of the 3D coordinates of n points and the parameters of m cameras, compute in
 * hx the prediction of the measurements, i.e. the projections of 3D points in the m images. The measurements
 * are returned in the order (hx_11^T, .. hx_1m^T, ..., hx_n1^T, .. hx_nm^T)^T, where hx_ij is the predicted
 * projection of the i-th point on the j-th camera.
 * Notice that depending on idxij, some of the hx_ij might be missing
 *
 */
static void img_projsRTS_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata)
{
    int i, j;
    int cnp, pnp, mnp;
    double *pa, *pb, *pqr, *pt, *ppt, *pmeas, *Kparms, *pr0, lrot[FULLQUATSZ], trot[FULLQUATSZ];
    //int n;
    int m, nnz;
    struct globs_ *gl;

    gl=(struct globs_ *)adata;
    cnp=gl->cnp; pnp=gl->pnp; mnp=gl->mnp;
    Kparms=gl->intrcalib;

    //n=idxij->nr;
    m=idxij->nc;
    pa=p; pb=p+m*cnp;

    for(j=0; j<m; ++j){
        /* j-th camera parameters */
        pqr=pa+j*cnp;
        pt=pqr+3; // quaternion vector part has 3 elements
        pr0=gl->rot0params+j*FULLQUATSZ; // full quat for initial rotation estimate
        _MK_QUAT_FRM_VEC(lrot, pqr);
        quatMultFast(lrot, pr0, trot); // trot=lrot*pr0

        nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */

        for(i=0; i<nnz; ++i){
            ppt=pb + rcsubs[i]*pnp;
            pmeas=hx + idxij->val[rcidxs[i]]*mnp; // set pmeas to point to hx_ij

            calcImgProjFullR(Kparms, trot, pt, ppt, pmeas); // evaluate Q in pmeas
            //calcImgProj(Kparms, pr0, pqr, pt, ppt, pmeas); // evaluate Q in pmeas
        }
    }
}

/* Given a parameter vector p made up of the 3D coordinates of n points and the parameters of m cameras, compute in
 * jac the jacobian of the predicted measurements, i.e. the jacobian of the projections of 3D points in the m images.
 * The jacobian is returned in the order (A_11, ..., A_1m, ..., A_n1, ..., A_nm, B_11, ..., B_1m, ..., B_n1, ..., B_nm),
 * where A_ij=dx_ij/db_j and B_ij=dx_ij/db_i (see HZ).
 * Notice that depending on idxij, some of the A_ij, B_ij might be missing
 *
 */
static void img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata)
{
    int i, j;
    int cnp, pnp, mnp;
    double *pa, *pb, *pqr, *pt, *ppt, *pA, *pB, *Kparms, *pr0;
    //int n;
    int m, nnz, Asz, Bsz, ABsz;
    struct globs_ *gl;

    gl=(struct globs_ *)adata;
    cnp=gl->cnp; pnp=gl->pnp; mnp=gl->mnp;
    Kparms=gl->intrcalib;

    //n=idxij->nr;
    m=idxij->nc;
    pa=p; pb=p+m*cnp;
    Asz=mnp*cnp; Bsz=mnp*pnp; ABsz=Asz+Bsz;

    for(j=0; j<m; ++j){
        /* j-th camera parameters */
        pqr=pa+j*cnp;
        pt=pqr+3; // quaternion vector part has 3 elements
        pr0=gl->rot0params+j*FULLQUATSZ; // full quat for initial rotation estimate

        nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */

        for(i=0; i<nnz; ++i){
            ppt=pb + rcsubs[i]*pnp;
            pA=jac + idxij->val[rcidxs[i]]*ABsz; // set pA to point to A_ij
            pB=pA  + Asz; // set pB to point to B_ij

            calcImgProjJacRTS(Kparms, pr0, pqr, pt, ppt, (double (*)[6])pA, (double (*)[3])pB); // evaluate dQ/da, dQ/db in pA, pB
        }
    }
}




void sba_driver( const std::vector< cv::Matx44d >& cam_poses,
        const std::vector< cv::Vec3d >& pts3d,
        const std::vector< std::vector< cv::Vec2d > >& pts2d,
        std::vector< cv::Mat >& Rsba,
        std::vector< cv::Mat >& Tsba )
{
    LOG_SCOPE("sba_driver");
    const int cnp = 6; //3 rot + 3 trans params
    const int pnp = 3; // xyz points
    const int mnp = 2; // uv reprojections

    double ical[5]; // intrinsic calibration matrix & temp. storage for its params
    double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
    int fixedcal, havedist, n, verbose=0;
    int nframes, numpts3D, numprojs, nvars;
    const int nconstframes=0;
    int i;

    nframes = static_cast<int>( cam_poses.size() );
    numpts3D = static_cast<int>( pts3d.size() );
    numprojs = numpts3D*nframes;

    // Sanity check
    if( pts2d.size() != nframes )
    {
        LOGE << "pts2d.size() != cam_poses.size()";
        return;
    }
    for( size_t camidx=0; camidx<nframes; ++camidx )
    {
        if( pts2d[camidx].size() != pts3d.size() )
        {
            LOGE << "pts2d[" << camidx << "].size() != pts3d.size()";
            return;
        }
    }

    std::vector< char > vmask( nframes * numpts3D, 1 ); // All 3D points are visible from all the cameras
    std::vector< double > motstruct( nframes*cnp + numpts3D*pnp );
    std::vector< double > initrot( nframes * FULLQUATSZ );
    std::vector< double > imgpts( numpts3D * nframes * mnp );


    // Fill motstruct and initrot
    size_t initrotidx=0;
    size_t motstructidx=0;
    double inaux[4];
    double outaux[4];
    // mot
    for( size_t camidx=0; camidx<nframes; ++camidx )
    {
        cv::Matx44d campose = cam_poses[camidx];
        cv::Vec4d camRq = rotation_mat_to_quaternion( campose );
        cv::Vec3d camT( campose(0,3), campose(1,3), campose(2,3) );

        initrot[ initrotidx++ ] = inaux[0] = camRq[0];
        initrot[ initrotidx++ ] = inaux[1] = camRq[1];
        initrot[ initrotidx++ ] = inaux[2] = camRq[2];
        initrot[ initrotidx++ ] = inaux[3] = camRq[3];

        quat2vec( inaux,4,outaux,3 );
        motstruct[ motstructidx++ ] = outaux[0];
        motstruct[ motstructidx++ ] = outaux[1];
        motstruct[ motstructidx++ ] = outaux[2];
        motstruct[ motstructidx++ ] = camT[0];
        motstruct[ motstructidx++ ] = camT[1];
        motstruct[ motstructidx++ ] = camT[2];
    }


    //struct
    for( size_t ptidx = 0; ptidx<numpts3D; ++ptidx )
    {
        const cv::Vec3d& p3d = pts3d[ptidx];
        motstruct[ motstructidx++ ] = p3d[0];
        motstruct[ motstructidx++ ] = p3d[1];
        motstruct[ motstructidx++ ] = p3d[2];
    }

    // Fill imgpts
    size_t imgptsidx=0;
    for( size_t ptidx = 0; ptidx<numpts3D; ++ptidx )
    {
        for( size_t camidx=0; camidx<nframes; ++camidx )
        {
            const cv::Vec2d& currpt2d = pts2d.at( camidx ).at( ptidx );
            imgpts[ imgptsidx++ ] = currpt2d[0];
            imgpts[ imgptsidx++ ] = currpt2d[1];
        }
    }


    {
        /* initialize the local rotation estimates to 0, corresponding to local quats (1, 0, 0, 0) */
        for(i=0; i<nframes; ++i){
            int j;

            j=(i+1)*cnp; // note the +1, below we move from right to left, assuming 3 parameters for the translation!
            motstruct[j-4]=motstruct[j-5]=motstruct[j-6]=0.0; // clear rotation
        }
    }

    /* set up globs structure */
    globs.cnp=cnp; globs.pnp=pnp; globs.mnp=mnp;
    globs.rot0params = &(initrot[0]);
    ical[0]=1.0;// f K[0]; // fu
    ical[1]=0.0;// cx K[2]; // u0
    ical[2]=0.0;// cyK[5]; // v0
    ical[3]=1.0;// aspect ratio K[4]/K[0]; // ar
    ical[4]=0.0;// skewness K[1]; // s
    globs.intrcalib=ical;
    fixedcal=1; /* use fixed intrinsics */
    havedist=0;
    globs.ptparams=0;
    globs.camparams=0;

    /* call sparse LM routine */
    opts[0]=SBA_INIT_MU; opts[1]=SBA_STOP_THRESH; opts[2]=SBA_STOP_THRESH;
    opts[3]=SBA_STOP_THRESH;
    opts[4]=0.0;

    {
        LOGI << "Running SBA using " << numpts3D << " 3D pts, " << nframes << " frames and " << numprojs << " image projections";
        LOGI << "Method SBA_MOTSTRUCT, expert driver, analytic Jacobian, fixed intrinsics";
        nvars=nframes*cnp+numpts3D*pnp;
        n=sba_motstr_levmar_x(numpts3D, 0, nframes, nconstframes, &(vmask[0]), &(motstruct[0]), cnp, pnp, &(imgpts[0]), /*covimgpts*/0, mnp,
                img_projsRTS_x,
                img_projsRTS_jac_x,
                (void *)(&globs), /*MAXITER2*/1000, verbose, opts, info);
    }


    if(n==SBA_ERROR) goto cleanup;

    {
        /* combine the local rotation estimates with the initial ones */
        for(i=0; i<nframes; ++i){
            double *v, qs[FULLQUATSZ], *q0, prd[FULLQUATSZ];

            /* retrieve the vector part */
            v = &(motstruct[0]) + (i+1)*cnp - 6; // note the +1, we access the motion parameters from the right, assuming 3 for translation!
            _MK_QUAT_FRM_VEC(qs, v);

            q0 = &(initrot[0]) +i*FULLQUATSZ;
            quatMultFast(qs, q0, prd); // prd=qs*q0

            /* copy back vector part making sure that the scalar part is non-negative */
            if(prd[0]>=0.0){
                v[0]=prd[1];
                v[1]=prd[2];
                v[2]=prd[3];
            }
            else{ // negate since two quaternions q and -q represent the same rotation
                v[0]=-prd[1];
                v[1]=-prd[2];
                v[2]=-prd[3];
            }
        }
    }

    LOGI << "SBA returned " << n << " in " << info[5] << " iter, reason " << info[6] << " error " << info[1]/numprojs << " (initial " << info[0]/numprojs << ")";


    // Extract the updated motstruct
    motstructidx=0;
    for( size_t camidx=0; camidx<nframes; ++camidx )
    {
        cv::Vec3d T;
        inaux[0] = motstruct[ motstructidx++];
        inaux[1] = motstruct[ motstructidx++];
        inaux[2] = motstruct[ motstructidx++];
        T[0] = motstruct[ motstructidx++ ];
        T[1] = motstruct[ motstructidx++ ];
        T[2] = motstruct[ motstructidx++ ];

        vec2quat( inaux, 3, outaux, 4 );
        cv::Vec4d q( outaux[0], outaux[1], outaux[2], outaux[3] );
        cv::Matx33d R = quaternion_to_rot_matrix(q);
        Rsba.push_back( (cv::Mat)R );
        Tsba.push_back( (cv::Mat)T );
    }

cleanup:
    /* just in case... */
    globs.intrcalib=0;
    globs.nccalib=0;
    globs.ncdist=0;
    globs.rot0params=0;
}
