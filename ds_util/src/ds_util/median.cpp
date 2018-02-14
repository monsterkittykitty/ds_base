#include "ds_util/median.h"

namespace ds_util
{
  void sort(int n, double ra[])
  {
    int l,j,ir,i;
    double rra;

    l=(n >> 1)+1;
    ir=n;
    for (;;) {
      if (l > 1)
	rra=ra[--l];
      else {
	rra=ra[ir];
	ra[ir]=ra[1];
	if (--ir == 1) {
	  ra[1]=rra;
	  return;
	}
      }
      i=l;
      j=l << 1;
      while (j <= ir) {
	if (j < ir && ra[j] < ra[j+1]) ++j;
	if (rra < ra[j]) {
	  ra[i]=ra[j];
	  j += (i=j);
	}
	else j=ir+1;
      }
      ra[i]=rra;
    }
  }

  double median(double x[],int n)
  {
    double store[100];
    int n2,n2p;
    int i ;
    double result ;
    for(i=0;i<n;i++)store[i+1]=x[i] ;
    sort(n,store);
    n2p=(n2=n/2)+1;
    result=(n % 2 ? store[n2p] : 0.5*(store[n2]+store[n2p]));
    return(result) ;
  }
}
