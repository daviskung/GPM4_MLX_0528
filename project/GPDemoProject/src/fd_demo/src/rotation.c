#include "define.h"

/************** interpolation constants and tables ***************/
#define  _DESCALE(x,n)			(((x) + (1 << ((n)-1))) >> (n))
#define I_WARP_SHIFT			10
#define I_WARP_MASK				((1 << I_WARP_SHIFT) - 1)
#define I_WARP_MUL_ONE_8U(x)	((x) << I_WARP_SHIFT)
#define I_WARP_DESCALE_8U(x)	_DESCALE((x), I_WARP_SHIFT*2)
#define I_WARP_CLIP_X(x)		((unsigned)(x) < (unsigned)src->width ? (x) : (x) < 0 ? 0 : src->width - 1)
#define I_WARP_CLIP_Y(y)		((unsigned)(y) < (unsigned)src->height ? (y) : (y) < 0 ? 0 : src->height - 1)

/****************************************************************************************\
*                                     WarpAffine                                         *
\****************************************************************************************/
static void WarpAffineBilinear(gpImage *src, gpImage *dst, int *matrix)
{
	int x, y, k;
	unsigned char *p_src, *p_dst;
    int A12 = matrix[1], b1 = matrix[2];
    int A22 = matrix[4], b2 = matrix[5];
	int A11 = matrix[0], A21 = matrix[3];
	int cn = src->ch;

	p_src = src->ptr;
	p_dst = dst->ptr;
	for( y = 0; y < dst->height; y++, p_dst += dst->widthStep )
    {
        int xs = A12*y + b1;
        int ys = A22*y + b2;

        for( x = 0; x < dst->width; x++ )
        {
            int ixs = xs + (x * A11);
            int iys = ys + (x * A21);

			// a, b is reminder for bilinear interpolation
            int a = ( ixs & I_WARP_MASK );
            int b = ( iys & I_WARP_MASK );

            int p0, p1;
            ixs >>= I_WARP_SHIFT;
            iys >>= I_WARP_SHIFT;

            if( (unsigned)ixs < (unsigned)(src->width - 1) &&
                (unsigned)iys < (unsigned)(src->height - 1) )
            {
				unsigned char* ptr = p_src + src->widthStep*iys + ixs*cn;

                for( k = 0; k < cn; k++ ) {
                    p0 = I_WARP_MUL_ONE_8U(ptr[k]) + a * (ptr[k+cn] - ptr[k]);
                    p1 = I_WARP_MUL_ONE_8U(ptr[k+src->widthStep]) + a * (ptr[k+cn+src->widthStep] - ptr[k+src->widthStep]);
                    p0 = I_WARP_DESCALE_8U(I_WARP_MUL_ONE_8U(p0) + b*(p1 - p0));
                    p_dst[x*cn+k] = (unsigned char)(p0);
                }
            }
            else if( (unsigned)(ixs+1) < (unsigned)(src->width+1) &&
                     (unsigned)(iys+1) < (unsigned)(src->height+1))
            {
                int x0 = I_WARP_CLIP_X( ixs );
                int y0 = I_WARP_CLIP_Y( iys );
                int x1 = I_WARP_CLIP_X( ixs + 1 );
                int y1 = I_WARP_CLIP_Y( iys + 1 );
                unsigned char* ptr0, *ptr1, *ptr2, *ptr3;

                ptr0 = p_src + y0*src->widthStep + x0*cn;
                ptr1 = p_src + y0*src->widthStep + x1*cn;
                ptr2 = p_src + y1*src->widthStep + x0*cn;
                ptr3 = p_src + y1*src->widthStep + x1*cn;

                for( k = 0; k < cn; k++ ) {
                    p0 = I_WARP_MUL_ONE_8U(ptr0[k]) + a * (ptr1[k] - ptr0[k]);
                    p1 = I_WARP_MUL_ONE_8U(ptr2[k]) + a * (ptr3[k] - ptr2[k]);
                    p0 = I_WARP_DESCALE_8U( I_WARP_MUL_ONE_8U(p0) + b*(p1 - p0) );
                    p_dst[x*cn+k] = (unsigned char)(p0);
                }
            }
            else
            {
                for( k = 0; k < cn; k++ ) {
                    p_dst[x*cn+k] = 0;
                }
            }
        }
    }
}


#define SINCOS_BITS 14
#define SINCOS_PRECISION	(1 << SINCOS_BITS)
#define SINCOS_CONST(A)		(((A) >= 0) ? ((int)((A)*(SINCOS_PRECISION)+0.5)) : ((int)((A)*(SINCOS_PRECISION)-0.5)))

#define SINCOS_MUL_C(A,B)	(((int)(A)*(int)(B)+(1 << (SINCOS_BITS-1))) >> SINCOS_BITS)


// cos(-PI/6 ~ PI/6) (interval: 1 degree)
static const short cos_tab[] = {
	SINCOS_CONST(0.86602540), SINCOS_CONST(0.87461971), SINCOS_CONST(0.88294759), SINCOS_CONST(0.89100652), SINCOS_CONST(0.89879405),
	SINCOS_CONST(0.90630779), SINCOS_CONST(0.91354546), SINCOS_CONST(0.92050485), SINCOS_CONST(0.92718385), SINCOS_CONST(0.93358043),
	SINCOS_CONST(0.93969262), SINCOS_CONST(0.94551858), SINCOS_CONST(0.95105652), SINCOS_CONST(0.95630476), SINCOS_CONST(0.96126170),
	SINCOS_CONST(0.96592583), SINCOS_CONST(0.97029573), SINCOS_CONST(0.97437006), SINCOS_CONST(0.97814760), SINCOS_CONST(0.98162718),
	SINCOS_CONST(0.98480775), SINCOS_CONST(0.98768834), SINCOS_CONST(0.99026807), SINCOS_CONST(0.99254615), SINCOS_CONST(0.99452190),
	SINCOS_CONST(0.99619470), SINCOS_CONST(0.99756405), SINCOS_CONST(0.99862953), SINCOS_CONST(0.99939083), SINCOS_CONST(0.99984770),
	SINCOS_CONST(1.00000000),
	SINCOS_CONST(0.99984770), SINCOS_CONST(0.99939083), SINCOS_CONST(0.99862953), SINCOS_CONST(0.99756405), SINCOS_CONST(0.99619470),
	SINCOS_CONST(0.99452190), SINCOS_CONST(0.99254615), SINCOS_CONST(0.99026807), SINCOS_CONST(0.98768834), SINCOS_CONST(0.98480775),
	SINCOS_CONST(0.98162718), SINCOS_CONST(0.97814760), SINCOS_CONST(0.97437006), SINCOS_CONST(0.97029573), SINCOS_CONST(0.96592583),
	SINCOS_CONST(0.96126170), SINCOS_CONST(0.95630476), SINCOS_CONST(0.95105652), SINCOS_CONST(0.94551858), SINCOS_CONST(0.93969262),
	SINCOS_CONST(0.93358043), SINCOS_CONST(0.92718385), SINCOS_CONST(0.92050485), SINCOS_CONST(0.91354546), SINCOS_CONST(0.90630779),
	SINCOS_CONST(0.89879405), SINCOS_CONST(0.89100652), SINCOS_CONST(0.88294759), SINCOS_CONST(0.87461971), SINCOS_CONST(0.86602540)};


static const short sin_tab[] = {
	SINCOS_CONST(-0.50000000), SINCOS_CONST(-0.48480962), SINCOS_CONST(-0.46947156), SINCOS_CONST(-0.45399050), SINCOS_CONST(-0.43837115),
	SINCOS_CONST(-0.42261826), SINCOS_CONST(-0.40673664), SINCOS_CONST(-0.39073113), SINCOS_CONST(-0.37460659), SINCOS_CONST(-0.35836795),
	SINCOS_CONST(-0.34202014), SINCOS_CONST(-0.32556815), SINCOS_CONST(-0.30901699), SINCOS_CONST(-0.29237170), SINCOS_CONST(-0.27563736),
	SINCOS_CONST(-0.25881905), SINCOS_CONST(-0.24192190), SINCOS_CONST(-0.22495105), SINCOS_CONST(-0.20791169), SINCOS_CONST(-0.19080900),
	SINCOS_CONST(-0.17364818), SINCOS_CONST(-0.15643447), SINCOS_CONST(-0.13917310), SINCOS_CONST(-0.12186934), SINCOS_CONST(-0.10452846),
	SINCOS_CONST(-0.08715574), SINCOS_CONST(-0.06975647), SINCOS_CONST(-0.05233596), SINCOS_CONST(-0.03489950), SINCOS_CONST(-0.01745241),
	SINCOS_CONST( 0.00000000),
	SINCOS_CONST( 0.01745241), SINCOS_CONST( 0.03489950), SINCOS_CONST( 0.05233596), SINCOS_CONST( 0.06975647), SINCOS_CONST( 0.08715574),
	SINCOS_CONST( 0.10452846), SINCOS_CONST( 0.12186934), SINCOS_CONST( 0.13917310), SINCOS_CONST( 0.15643447), SINCOS_CONST( 0.17364818),
	SINCOS_CONST( 0.19080900), SINCOS_CONST( 0.20791169), SINCOS_CONST( 0.22495105), SINCOS_CONST( 0.24192190), SINCOS_CONST( 0.25881905),
	SINCOS_CONST( 0.27563736), SINCOS_CONST( 0.29237170), SINCOS_CONST( 0.30901699), SINCOS_CONST( 0.32556815), SINCOS_CONST( 0.34202014),
	SINCOS_CONST( 0.35836795), SINCOS_CONST( 0.37460659), SINCOS_CONST( 0.39073113), SINCOS_CONST( 0.40673664), SINCOS_CONST( 0.42261826),
	SINCOS_CONST( 0.43837115), SINCOS_CONST( 0.45399050), SINCOS_CONST( 0.46947156), SINCOS_CONST( 0.48480962), SINCOS_CONST( 0.50000000)};



// angle is radian not degree
static void twoDRotationMatrix( short x, short y, int angle, int scale, int m[6], int inv_m[6] )
{
    int alpha, beta;

	if ((angle < -30) || (angle > 30)) {
		angle = 0;
		DBG_PRINT("!!!!!ERROR!!!!!\r\n");
	}

    alpha = SINCOS_MUL_C(cos_tab[angle + 30], scale);
    beta =  SINCOS_MUL_C(sin_tab[angle + 30], scale);

    inv_m[0] = alpha;
    inv_m[1] = beta;
    inv_m[2] = (I_WARP_MUL_ONE_8U(1) - alpha) * x - beta * y;
    inv_m[3] = -beta;
    inv_m[4] = alpha;
    inv_m[5] = beta * x + (I_WARP_MUL_ONE_8U(1) - alpha) * y;

	// [R|t] -> [R^-1 | -(R^-1)*t]
	/*
	// get inverseA
	ad_bc = m2[0] * m2[4] - m2[1] * m2[3];
	ad_bc = 1.0/ad_bc;

	matrix[0] = ad_bc * m2[4];
	matrix[1] = ad_bc * -1.0 * m2[1];
	matrix[3] = ad_bc * -1.0 * m2[3];
	matrix[4] = ad_bc * m2[0];

	// -InverseA * b
	matrix[2] = -(matrix[0]*m2[2] + matrix[1]*m2[5]);
	matrix[5] = -(matrix[3]*m2[2] + matrix[4]*m2[5]);
	*/

	// floating -> fix ( Q6.10 )
	m[0] = alpha;
	m[1] = -beta;
	m[2] = -x * alpha + y * beta + I_WARP_MUL_ONE_8U(x);
	m[3] = beta;
	m[4] = alpha;
	m[5] = -x * beta - y * alpha + I_WARP_MUL_ONE_8U(y);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Image Rotation function
// input: src
// output: dst
// pt: the center point of the image
// angle: radian not degree
// scale: image scaling
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void image_rotate(gpImage *src, gpImage *dst, gpPoint *pt, int angle, int scale)
{
	int m[6], inv_m[6];

	twoDRotationMatrix(pt->x, pt->y, angle, scale, m, inv_m);
	WarpAffineBilinear(src, dst, m);
}
