#ifndef __LIBGT_H__
#define __LIBGT_H__




#ifdef __cplusplus
extern "C" {
#endif


    void gt_create_population( double *x, int size );
    void gt_iidyn( const double *A, double *x, int size, double* toll, int* max_iters );


#ifdef __cplusplus
};
#endif




#endif
