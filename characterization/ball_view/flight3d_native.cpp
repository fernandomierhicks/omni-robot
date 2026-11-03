// Numpy reference: flight3d.fit / trajectory. No OpenCV, ROS or board headers.
#include <algorithm>
#include <cmath>
#include <vector>

static void residual(const double *s,const double *obs,int n,const double *intr,
                     const double *g,double drag,std::vector<double>&r) {
    double p[3]={s[0],s[1],s[2]},v[3]={s[3],s[4],s[5]},now=0;
    r.resize(3*n);
    for(int i=0;i<n;i++) {
        const double *o=obs+6*i;
        const int steps=std::max(1,(int)std::ceil((o[0]-now)/.012));
        const double dt=(o[0]-now)/steps;
        for(int j=0;j<steps;j++) {
            const double speed=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
            double mid[3];
            for(int a=0;a<3;a++)mid[a]=v[a]+.5*dt*(g[a]-drag*speed*v[a]);
            const double ms=std::sqrt(mid[0]*mid[0]+mid[1]*mid[1]+mid[2]*mid[2]);
            for(int a=0;a<3;a++){p[a]+=dt*mid[a];v[a]+=dt*(g[a]-drag*ms*mid[a]);}
        }
        const double z=std::max(p[2],.08);
        r[3*i]=(intr[0]*p[0]/z+intr[2]-o[1])/o[4];
        r[3*i+1]=(intr[1]*p[1]/z+intr[3]-o[2])/o[4];
        r[3*i+2]=(p[2]-o[3])/o[5];
        now=o[0];
    }
}
static double loss(const std::vector<double>&r,double cap) {
    double total=0;for(double a:r)total+=std::min(a*a,cap);return total/r.size();
}
static bool solve6(double a[6][7],double *x) {
    for(int i=0;i<6;i++) {
        int pivot=i;for(int j=i+1;j<6;j++)if(std::abs(a[j][i])>std::abs(a[pivot][i]))pivot=j;
        if(std::abs(a[pivot][i])<1e-12)return false;
        for(int k=i;k<7;k++)std::swap(a[i][k],a[pivot][k]);
        double d=a[i][i];for(int k=i;k<7;k++)a[i][k]/=d;
        for(int j=0;j<6;j++)if(j!=i){d=a[j][i];for(int k=i;k<7;k++)a[j][k]-=d*a[i][k];}
    }
    for(int i=0;i<6;i++)x[i]=a[i][6];return true;
}
#ifdef _WIN32
#define BUCKET_API extern "C" __declspec(dllexport)
#else
#define BUCKET_API extern "C"
#endif
BUCKET_API int bucket_fit(const double *obs,int n,const double *intr,const double *g,double drag,double *out) {
    if(n<3)return 0;
    double st=0,stt=0,sp[3]={},stp[3]={};
    for(int i=0;i<n;i++) {
        const double *o=obs+6*i;double t=o[0];st+=t;stt+=t*t;
        double p[3]={(o[1]-intr[2])/intr[0]*o[3],(o[2]-intr[3])/intr[1]*o[3],o[3]};
        for(int k=0;k<3;k++){p[k]-=.5*t*t*g[k];sp[k]+=p[k];stp[k]+=t*p[k];}
    }
    double den=n*stt-st*st;if(std::abs(den)<1e-12)return 0;
    double s[6];for(int k=0;k<3;k++){s[k+3]=(n*stp[k]-st*sp[k])/den;s[k]=(sp[k]-s[k+3]*st)/n;}
    const double eps[6]={.001,.001,.002,.01,.01,.01};
    std::vector<double> r,delta[6],candidate;
    for(int iteration=0;iteration<5;iteration++) {
        residual(s,obs,n,intr,g,drag,r);
        for(int k=0;k<6;k++) {
            double ss[6];std::copy(s,s+6,ss);ss[k]+=eps[k];residual(ss,obs,n,intr,g,drag,delta[k]);
            for(int j=0;j<3*n;j++)delta[k][j]=(delta[k][j]-r[j])/eps[k];
        }
        double normal[6][7]={};
        for(int j=0;j<3*n;j++) {
            const double w=std::min(1.,2.5/std::max(std::abs(r[j]),1e-6));
            for(int k=0;k<6;k++) {
                normal[k][6]-=delta[k][j]*w*r[j];
                for(int l=0;l<6;l++)normal[k][l]+=delta[k][j]*w*delta[l][j];
            }
        }
        for(int k=0;k<6;k++)normal[k][k]+=1e-5;
        double step[6];if(!solve6(normal,step))return 0;
        double norm=0;for(double a:step)norm+=a*a;norm=std::sqrt(norm);
        if(norm>3)for(double &a:step)a*=3/norm;
        bool accepted=false;const double old=loss(r,25);
        for(double f:{1.,.4,.1}) {
            double ss[6];for(int k=0;k<6;k++)ss[k]=s[k]+f*step[k];
            if(ss[2]<.15 || ss[3]*ss[3]+ss[4]*ss[4]+ss[5]*ss[5]>625)continue;
            residual(ss,obs,n,intr,g,drag,candidate);
            if(loss(candidate,25)<=old){std::copy(ss,ss+6,s);accepted=true;break;}
        }
        if(!accepted || norm<.0005)break;
    }
    residual(s,obs,n,intr,g,drag,r);
    std::copy(s,s+6,out);out[6]=std::sqrt(loss(r,100));
    for(int k=0;k<7;k++)if(!std::isfinite(out[k]))return 0;
    return 1;
}

BUCKET_API void bucket_path(const double *states,int batches,const double *times,int count,
                            const double *g,double drag,double *out) {
    for(int b=0;b<batches;b++) {
        const double *state=states+6*b;
        double p[3]={state[0],state[1],state[2]},v[3]={state[3],state[4],state[5]},now=0;
        for(int i=0;i<count;i++) {
            const int steps=std::max(1,(int)std::ceil((times[i]-now)/.012));
            const double dt=(times[i]-now)/steps;
            for(int j=0;j<steps;j++) {
                const double speed=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
                double mid[3];
                for(int a=0;a<3;a++)mid[a]=v[a]+.5*dt*(g[a]-drag*speed*v[a]);
                const double ms=std::sqrt(mid[0]*mid[0]+mid[1]*mid[1]+mid[2]*mid[2]);
                for(int a=0;a<3;a++){p[a]+=dt*mid[a];v[a]+=dt*(g[a]-drag*ms*mid[a]);}
            }
            for(int a=0;a<3;a++)out[(b*count+i)*3+a]=p[a];
            now=times[i];
        }
    }
}
