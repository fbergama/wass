#include "gt.h"
#include <limits>
#include <iostream>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <iomanip>


namespace gt
{

   template <typename T>
   inline void mult(const T *A,const T *x, int size, T *y)
   {
      const T *ptr;
      double yv;
      for(int j=0;j<size;++j,++y){
         ptr=x;
         //*y=0;
         yv = 0;
         for(int i=0;i<size;++i,++ptr,++A)
            //*y+=*A**ptr;
            yv += *A * *ptr;
         *y = static_cast<T>(yv);
      }
   }

   template <typename T>
   inline void mult_m(const T *A,const T *x, int size, T *y, const bool *mask)
   {
      const T *ptr;
      double yv;
      const bool *m1=mask, *m2;
      for(int j=0;j<size;++j,++y,++m1)
      {
         if (!*m1)
         {
            ptr=x;
            //*y=0;
            yv = 0;
            m2=mask;
            for(int i=0;i<size;++i,++ptr,++A,++m2)
               if(!*m2)
                  //*y+=*A**ptr;
                  yv += *A * *ptr;
            *y = static_cast<T>(yv);
         }
         else
            A+=size;
      }
   }


   template <typename T>
   inline void simplexify(T *x, int size){
      T sum=0;
      T *ptr=x;
      for(int i=0;i<size;++i,++ptr)
         if(*ptr>=0)
            sum+=*ptr;
      else
         *ptr=0;
      for(int i=0;i<size;++i,++x)
         *x/=sum;
   }

   template <typename T>
   inline void simplexify_m(T *x, int size, const bool *mask){
      T sum=0;
      T *ptr=x;
      const bool *m=mask;
      for(int i=0;i<size;++i,++ptr,++m)
         if(!*m){
         if(*ptr>=0)
            sum+=*ptr;
         else
            *ptr=0;
      }
      m=mask;
      for(int i=0;i<size;++i,++x,++m)
         if(!*m)
            *x/=sum;
   }

   template <typename T>
   inline double dot(const T *x, const T *y, int size){
      double sum=0;
      for(int i=0;i<size;++i,++x,++y)
         sum+=*x**y;
      return sum;
   }

   template <typename T>
   inline double dot_m(const T *x, const T *y, int size, const bool *mask){
      double sum=0;
      for(int i=0;i<size;++i,++x,++y,++mask)
         if(!*mask)
            sum+=*x**y;
      return sum;
   }

   template <typename T>
   inline void scale(T *x, int size, double c){
      for(int i=0;i<size;++i,++x)
         *x*=static_cast<T>( c );
   }

   inline void mul_scaled(const double *x, double *y,int size, double c){
      for(int i=0;i<size;++i,++x,++y)
         *y*=*x*c;
   }

   template <typename T>
   inline void scale_m(T *x, int size, double c, const bool *mask){
      for(int i=0;i<size;++i,++x,++mask)
         if(!*mask)
            *x*=static_cast<T>(c);
   }

   template <typename T>
   inline void linear_comb(const T *x, T *y, int size,double alfa){
      for(int i=0;i<size;++i,++x,++y)
         *y=static_cast<T>( alfa*(*x-*y)+*y );
   }

   template <typename T>
   inline void linear_comb_m(const T *x, T *y, int size,double alfa,const bool *mask){
      for(int i=0;i<size;++i,++x,++y,++mask)
         if(!*mask)
            *y=static_cast<T>( alfa*(*x-*y)+*y );
   }

   template <typename T>
   inline double nash_error(const T *x, const T *Ax, double xAx, int size)
   {
      double sum=0;
      const T *p_x=x;
      const T *p_Ax=Ax;
      double tmp;
      for(int i=0;i<size;++i,++p_x,++p_Ax){
         tmp=xAx-*p_Ax;
         if(tmp>*p_x)
            tmp=*p_x;
         sum+=tmp*tmp;
      }
      return sum;
   }

   template <typename T>
   inline double nash_error_m(const T *x, const T *Ax, double xAx, int size, const bool *mask)
   {
      double sum=0;
      const T *p_x=x;
      const T *p_Ax=Ax;
      double tmp;
      for(int i=0;i<size;++i,++p_x,++p_Ax,++mask)
         if(!*mask)
         {
            tmp=xAx-*p_Ax;
            if(tmp>*p_x)
               tmp=*p_x;
            sum+=tmp*tmp;
         }
      return sum;
   }

   template <typename T>
   inline double selectStrategy(const T *Ax, const T *x, int size, double &delta, int &idx)
   {
      const T *ptr1,*ptr2;
      double maxv = -std::numeric_limits<double>::infinity();
      double minv = std::numeric_limits<double>::infinity();

      int max_idx=-1,min_idx=-1;
      ptr1=Ax;
      ptr2=x;

      for(int i=0;i<size;++i,++ptr1,++ptr2){
         if(*ptr1>maxv){
            maxv=*ptr1;
            max_idx=i;
         }
         if(*ptr2>0&&*ptr1<minv){
            minv=*ptr1;
            min_idx=i;
         }
      }
      double xAx=dot(Ax,x,size);

      maxv-=xAx;
      minv=xAx-minv;

      idx=max_idx;
      delta=maxv;
      if(maxv<minv){
         idx=min_idx;
         delta=-minv;
      }

      //return max+min;
      double error=nash_error(x,Ax,xAx,size);
      //std::cout << error << std::endl;
      return error;
   }

   template <typename T>
   inline double selectStrategy_m(const T *Ax, const T *x, int size, double &delta, int &idx, const bool *mask)
   {
      const T *ptr1,*ptr2;
      double maxv=-std::numeric_limits<double>::infinity();
      double minv=std::numeric_limits<double>::infinity();

      int max_idx=-1,min_idx=-1;
      ptr1=Ax;
      ptr2=x;
      const bool *ptr_b=mask;

      for(int i=0;i<size;++i,++ptr1,++ptr2,++ptr_b)
         if(!*ptr_b){
         if(*ptr1>maxv){
            maxv=*ptr1;
            max_idx=i;
         }
         if(*ptr2>0&&*ptr1<minv){
            minv=*ptr1;
            min_idx=i;
         }
      }

      //if(max_idx<0||min_idx<0)return 0;
      double xAx=dot_m(Ax,x,size,mask);

      maxv-=xAx;
      minv=xAx-minv;

      idx=max_idx;
      delta=maxv;
      if(maxv<minv){
         idx=min_idx;
         delta=-minv;
      }

      double error=nash_error_m(x,Ax,xAx,size,mask);
      //std::cout << error << std::endl;
      return error;
      //return max+min;
   }


   void print_masked(const double *x, int size, const bool *mask){
      for(int i=0;i<size;++i,++x,++mask)
         if(*x>0&&!*mask)
            std::cout << i << ":" << *x << ", ";
      std::cout << std::endl;
   }


   //
   // Same as iidyn(), but a mask can be passed to exclude strategies
   //
   std::pair<int,double> iidyn_m(const float *A, float *x, int size, float toll, int max_iters, const bool *mask)
   {
      int niter=0;
      /* Calculate Ax */
      float *Ax=new float[size];
      simplexify_m(x,size,mask);
      mult_m(A,x,size,Ax,mask);

      /* OCCHIO */
      toll*=toll;

      double delta = 0.;
      double err = std::numeric_limits<double>::max();

      while(niter < max_iters)
      {
         int idx=-1;

         err = selectStrategy_m(Ax,x,size,delta,idx,mask);
         if (err < toll)
            break;

         double den=A[idx*(size+1)]-Ax[idx]-delta;
         bool do_remove=false;
         double mu,tmp;
         if(delta>=0)
         {
            mu=1;
            if(den<0)
            {
               tmp=-delta/den;
               if(mu>tmp)mu=tmp;
               if(mu<0)mu=0;
            }
         }
         else
         {
            //mu=1.0-1.0/(1-x[idx]);
            mu=x[idx]/(x[idx]-1);
            do_remove=true;
            if(den<0)
            {
               tmp=-delta/den;
               if(mu<tmp)
               {
                  mu=tmp;
                  do_remove=false;
               }
               if(mu>0)mu=0;
            }
         }
         scale_m(x,size,1-mu,mask);
         x[idx]=(float)( do_remove?0:(double)x[idx]+mu );

         simplexify_m(x,size,mask);

         linear_comb_m(A+idx*size, Ax, size, mu, mask);

         ++niter;
      }

      /* TO REMOVE !!! */
      /*
      double xAx=dot_m(Ax,x,size,mask);
      for(int i=0;i<size;++i)
         x[i]=Ax[i]/xAx>0.8?1:0;
         */

      delete[] Ax;

      return std::make_pair(niter,err);
   }


   void print(const double *x, int size){
      for(int i=0;i<size;++i,++x)
         if(*x>0)
            std::cout << i << ":" << *x << ", ";
      std::cout << std::endl;
   }


   void print_m(const double *A, int size){
      for(int j=0;j<size;j++){
         for(int i=0;i<size;++i,++A)
            std::cout << *A << " ";
         std::cout << std::endl;
      }
   }

   bool seeded=false;

   void mrand(double *A, int size, double offset, double density){
      if(!seeded){srand((unsigned int)time(NULL)); seeded=true;}
      for(int j=0;j<size;++j){
         A[j*size+j]=0;
         for(int i=j+1;i<size;++i)
            A[j*size+i]=A[i*size+j]=(double)rand()/RAND_MAX<=density?offset+(double)rand()/RAND_MAX:0;
      }
   }

   void vrand(double *x, int size){
      if(!seeded){srand( (unsigned int)time(NULL)); seeded=true;}
      for(int i=0;i<size;++i,++x)
         *x=(double)rand()/RAND_MAX;
   }

   template <typename T>
   inline void set_m(T *x,int size, double c,bool *mask){
      for(int j=0;j<size;++j,++mask,++x)
         if(!*mask)
            *x=static_cast<T>(c);
   }

   int clustering(const float *A, int *clusters, int size, int k){
      bool *mask=new bool[size];
      bool *ptr_b=mask;
      int n_clusters=0;
      //int *ptr_i=clusters;
      for(int i=0;i<size;++i,++ptr_b){
         *ptr_b=false;
         //*ptr_i=-1;
      }
      float *x=new float[size], *ptr_d;
      int n=size;
      for(int i=0;i<k&&n>0;++i){
         set_m(x,size,1.0/n,mask);
         int niter=iidyn_m(A,x,size,1E-7f,100000,mask).first;



         int cluster_size=n;
         ptr_d=x;
         ptr_b=mask;
         double max=0;
         int max_idx=-1;
         for(int j=0;j<size;++j,++ptr_d,++ptr_b)
            if(!*ptr_b&&*ptr_d>0){
            *ptr_b=true;
            if(*ptr_d>max){
               max=*ptr_d;
               max_idx=j;
            }
            //*ptr_i=i;
            --n;
         }
         *clusters++=max_idx;
         ++n_clusters;

         //for(int j=0;j<size;j++)std::cout << mask[j] << " " ;
         //std::cout << std::endl;

         std::cout << "cluster " << i << " iter="<<niter << " size="<<(cluster_size-n)<<std::endl;
      }

      delete[] mask;
      delete[] x;
      return n_clusters;
   }

   int clustering_noreass(const float *A, int *clusters, int size, int k){
      bool *mask=new bool[size];
      bool *ptr_b=mask;
      int n_clusters=0;
      //int *ptr_i=clusters;
      for(int i=0;i<size;++i,++ptr_b){
         *ptr_b=false;
         clusters[i]=0;
         //*ptr_i=-1;
      }
      float *x=new float[size], *ptr_d;
      int n=size;
      for(int i=0;i<k&&n>0;++i){
         set_m(x,size,1.0/n,mask);
         int niter=iidyn_m(A,x,size,1E-5f,100000,mask).first;

         int cluster_size=n;
         ptr_d=x;
         ptr_b=mask;

         int *p_clusters=clusters;

         for(int j=0;j<size;++j,++ptr_d,++ptr_b,++p_clusters)
            if(!*ptr_b&&*ptr_d>0){
            *ptr_b=true;
            *p_clusters=i+1;
            //*ptr_i=i;
            --n;
         }
         ++n_clusters;

         //for(int j=0;j<size;j++)std::cout << mask[j] << " " ;
         //std::cout << std::endl;

         std::cout << "cluster " << i << " iter="<<niter << " size="<<(cluster_size-n)<<std::endl;
      }

      delete[] mask;
      delete[] x;
      return n_clusters+1;
   }


   int repdyn(const double *A, double *x, int size, double toll){
      toll*=toll;
      double *Ax=new double[size];
      double xAx;
      int niter=0;
      simplexify(x,size);
      while(true){
         mult(A,x,size,Ax);
         xAx=dot(Ax,x,size);
         if(nash_error(x,Ax,xAx,size)<toll)break;
         mul_scaled(Ax,x,size,1.0/xAx);
         ++niter;
      }
      delete[] Ax;
      return niter;
   }

   int repdyn_v(const double *A, double *x, int size, double toll, double &error){
      toll*=toll;
      double *Ax=new double[size];
      double *xold=new double[size];
      double xAx=0;
      int niter=0;
      simplexify(x,size);
      double vel=2*toll;
      double tmp;
      while(vel>toll){
         mult(A,x,size,Ax);
         xAx=dot(Ax,x,size);
         memcpy(xold,x,sizeof(double)*size);
         mul_scaled(Ax,x,size,1.0/xAx);
         ++niter;
         vel=0;
         for(int i=0;i<size;++i){
            tmp=x[i]-xold[i];
            vel+=tmp*tmp;
         }
      }
      error=sqrt(nash_error(x,Ax,xAx,size));
      delete[] Ax;
      delete[] xold;
      return niter;
   }

}




//
// Infection-immunization dynamics
//
void gt_iidyn( const double *A, double *x, int size, double* toll, int* max_iters )
{
  int niter=0;
  /* Calculate Ax */
  double *Ax=new double[size];
  gt::simplexify(x,size);
  gt::mult(A,x,size,Ax);

  /*** OCCHIO **/
  (*toll)*=(*toll);

  //const std::string demfile(plot_fname+".dem");
  //std::ofstream dem(demfile.c_str());


  double delta = 0.;
  double err = std::numeric_limits<double>::max();

  while (niter < *max_iters)
  {
     int idx=-1;

     err = gt::selectStrategy(Ax,x,size,delta,idx);
     if (err < *toll)
        break;

     double den=A[idx*(size+1)]-Ax[idx]-delta;
     bool do_remove=false;
     double mu,tmp;
     if(delta>=0)
     {
        mu=1;
        if(den<0)
        {
           tmp=-delta/den;
           if(mu>tmp)mu=tmp;
           if(mu<0)mu=0;
        }
     }
     else
     {
        //mu=1.0-1.0/(1-x[idx]);
        mu=x[idx]/(x[idx]-1);
        do_remove=true;
        if(den<0)
        {
           tmp=-delta/den;
           if(mu<tmp)
           {
              mu=tmp;
              do_remove=false;
           }
           if(mu>0)mu=0;
        }
     }
     gt::scale(x,size,1-mu);
     x[idx]= do_remove?0.0:x[idx]+mu;

     gt::simplexify(x,size);

     gt::linear_comb(A+idx*size, Ax, size, mu);

     ++niter;
  }


  //dem.close();
  // gnuplot plotgif.gnuplot
  // cd plots
  // for f in *gif ; do convert -quality 100 $f `basename $f gif`jpg; done
  // mencoder "mf://*.jpg" -mf fps=10 -o plot.avi -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=800

  delete[] Ax;

  *max_iters = niter;
  *toll = err;
}




extern void gt_create_population( double *x, int size ) {
    const double delta_percent = size*0.03f; /* +- 3% */

    double sum=0.0f;
    for( int i=0; i<size; ++i ) {
        x[i] = 1.0f + rand()/RAND_MAX * delta_percent;
        sum += x[i];
    }
    for( int i=0; i<size; ++i ) {
        x[i]/=sum;
    }
}
