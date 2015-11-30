/**
	 @file
    ta.jit.kinect2 - reads rgb and depth matrix from Kinect v2
    using libfreenect2
 
    @ingroup	examples
	@see		ta.jit.kinect2
 
	Copyright 2015 - Tiago Ângelo aka p1nh0 (p1nh0.c0d1ng@gmail.com)*/

#include "jit.common.h"

// Libfreenect2 includes
#include <iostream>
#include <signal.h>
#include <libfreenect2.hpp>
#include <frame_listener_impl.h>
#include <registration.h>
#include <packet_pipeline.h>
//#include <logger.h>

// matrix dimensions
#define RGB_WIDTH 1920
#define RGB_HEIGHT 1080
#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424

// Our Jitter object instance data
typedef struct _ta_jit_kinect2 {
	t_object	ob;
    long		depth_processor;	// TA: depth_processor attribute -> 0=CPU, 1=OpenGL, 2=OpenCL
    libfreenect2::Freenect2 freenect2;
//    std::string *serial;
    libfreenect2::Freenect2Device *device; // TA: declare freenect2 device
    libfreenect2::PacketPipeline *pipeline; // TA: declare packet pipeline
    
    libfreenect2::SyncMultiFrameListener *depth_listener; //TA: depth frame listener
    libfreenect2::FrameMap *depth_frame; // TA: depth frames
    size_t framecount;
    t_bool isOpen;
} t_ta_jit_kinect2;


// prototypes
BEGIN_USING_C_LINKAGE
t_jit_err		ta_jit_kinect2_init				(void);
t_ta_jit_kinect2	*ta_jit_kinect2_new				(void);
void			ta_jit_kinect2_free				(t_ta_jit_kinect2 *x);
t_jit_err		ta_jit_kinect2_matrix_calc		(t_ta_jit_kinect2 *x, void *inputs, void *outputs);
void ta_jit_kinect2_open (t_ta_jit_kinect2 *x); // TA: declare "open" method
void ta_jit_kinect2_close (t_ta_jit_kinect2 *x); // TA: declare "close" method
void depth_callback (t_ta_jit_kinect2 *x, char *out_data, t_jit_matrix_info *minfo); // TA: declare private "depth_callback" method that listens for new depth frames when available
void copy_depth_data (t_ta_jit_kinect2 *x, char *out_data, t_jit_matrix_info *minfo); // TA: declare private "copy_depth_data" method to copy depth data from Kinect to our output depth matrix

// TA: ta.jit.kinect2 doesn't make any ndim calculation (there are no pixels being changed!)
//void			ta_jit_kinect2_calculate_ndim	(t_ta_jit_kinect2 *x, long dim, long *dimsize, long planecount, t_jit_matrix_info *in_minfo, char *bip, t_jit_matrix_info *out_minfo, char *bop);
END_USING_C_LINKAGE


// globals TA: GLOBAL CLASS VARIABLE (class information resides here)
static void *s_ta_jit_kinect2_class = NULL;
//std::string serial; //TA: not needed ?

/************************************************************************************/
// INIT
t_jit_err ta_jit_kinect2_init(void)
{
	long			attrflags = JIT_ATTR_GET_DEFER_LOW | JIT_ATTR_SET_USURP_LOW;
	t_jit_object	*attr;
	t_jit_object	*mop;

	s_ta_jit_kinect2_class = jit_class_new("ta_jit_kinect2", (method)ta_jit_kinect2_new, (method)ta_jit_kinect2_free, sizeof(t_ta_jit_kinect2), 0);

	// add matrix operator (mop)
	mop = (t_jit_object *)jit_object_new(_jit_sym_jit_mop, 0, 2); // args are  num inputs and num outputs | TA: added a second outlet, thus having depth, rgb and dumpout outlets
    jit_class_addadornment(s_ta_jit_kinect2_class, mop);

	// add method(s)
	jit_class_addmethod(s_ta_jit_kinect2_class, (method)ta_jit_kinect2_matrix_calc, "matrix_calc", A_CANT, 0);
    jit_class_addmethod(s_ta_jit_kinect2_class, (method)ta_jit_kinect2_open, "open", 0);
    jit_class_addmethod(s_ta_jit_kinect2_class, (method)ta_jit_kinect2_close, "close", 0);
    
	// add attribute(s)
	attr = (t_jit_object *)jit_object_new(_jit_sym_jit_attr_offset,
										  "depth_processor",
										  _jit_sym_long,
										  attrflags,
										  (method)NULL, (method)NULL,
										  calcoffset(t_ta_jit_kinect2, depth_processor));
	jit_class_addattr(s_ta_jit_kinect2_class, attr);

	// finalize class (REGISTER)
	jit_class_register(s_ta_jit_kinect2_class);
	return JIT_ERR_NONE;
}


/************************************************************************************/
// Object Life Cycle : CONSTRUCTOR

t_ta_jit_kinect2 *ta_jit_kinect2_new(void)
{
	t_ta_jit_kinect2	*x = NULL;
    // TA: ALLOCATE OBJECT STRUCT (only initializes object struct)
	x = (t_ta_jit_kinect2 *)jit_object_alloc(s_ta_jit_kinect2_class);
    // TA: initialize other data or structs
    if (x) {
		x->depth_processor = 2; //TA: default depth-processor is OpenCL
        x->freenect2 = *new libfreenect2::Freenect2();
        x->device = 0; //TA: init device
        x->pipeline = 0; //TA: init pipeline
//        serial.assign("005867245247");
        x->isOpen = false;
	}
	return x;
}

// DESTRUCTOR
void ta_jit_kinect2_free(t_ta_jit_kinect2 *x)
{
	;	// TA: close kinect device and free allocated resources (except for the object struct)
}


/************************************************************************************/
// Methods bound to input/inlets

//TA: open kinect device
void ta_jit_kinect2_open(t_ta_jit_kinect2 *x){
    if (x->isOpen == true) {
        return; // TA: ignore "open" message if a device is already open
    }
    post("reaching for Kinect2 device"); // TA: insert "open" method here
    
    // TA: check for connected devices
    if (x->freenect2.enumerateDevices() == 0) {
        post("no device connected!");
        return; // TA: exit open() method if no device is connected
    }
    
    post("reaching kinect devices...");
    if(!x->pipeline){
        x->pipeline = new libfreenect2::OpenCLPacketPipeline();
        post("creating OpenCL packet pipeline");
    }
    if(x->pipeline){
        x->device = x->freenect2.openDefaultDevice();
        post("opened default Kinect device");
    }
    if(x->device == 0){
        post("failed to open device...");
        return;
    }
    // TA: start device
    x->depth_listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Depth);
    x->device->setIrAndDepthFrameListener(x->depth_listener);
    x->depth_frame = new libfreenect2::FrameMap[libfreenect2::Frame::Type::Depth];
    x->device->start();
    x->framecount = 0; // TA: init framecount
    x->isOpen = true;
}
//TA: close kinect device
void ta_jit_kinect2_close(t_ta_jit_kinect2 *x){
    post("closing...");
    if (x->isOpen == false) {
        return; // quit close method if no device is open
    }
    x->device->stop();
    x->device->close();
    
    x->depth_listener->release(*x->depth_frame); // TA: just in case...
    x->depth_listener = NULL;
    x->device = 0; //TA: init device
    x->pipeline = 0; //TA: init pipeline
    x->isOpen = false;
}

//TA: private depth_callback method
void depth_callback(t_ta_jit_kinect2 *x, char *out_data, t_jit_matrix_info *minfo){
    x->depth_listener->waitForNewFrame(*x->depth_frame);
//    if(x->depth_callback->onNewFrame(libfreenect2::Frame::Type::Depth, x->depth_frame)){
//        x->framecount++; // TA: I don't think we need this variable!!!!
//        
//    }
    copy_depth_data(x, out_data, minfo);

    x->depth_listener->release(*x->depth_frame);
}

//TA: private copy_depth_data method
void copy_depth_data(t_ta_jit_kinect2 *x, char *out_data, t_jit_matrix_info *minfo){
    
    std::cout << "dimcount : " << minfo->dimcount << std::endl;
    std::cout << "dim[0] : " << minfo->dim[0] << std::endl;
    std::cout << "dim[1] : " << minfo->dim[1] << std::endl;
    std::cout << "dim stride size : " << sizeof(minfo->dimstride) << std::endl;
    std::cout << "dim stride 0 (data type in bytes): " << minfo->dimstride[0] << std::endl;
    std::cout << "dim stride 1 (data type in bytes x dim[0]): " << minfo->dimstride[1] << std::endl;

    libfreenect2::Frame *frame = (*x->depth_frame)[libfreenect2::Frame::Depth];
    float *frame_data = (float *)frame->data;

    float value;
    for (int yPos = 0; yPos < DEPTH_HEIGHT; yPos++){
        for (int xPos = 0; xPos < DEPTH_WIDTH; xPos++){
            value = frame_data[yPos*DEPTH_WIDTH+xPos];
            std::cout << "depth value at x="<< xPos << " and y =" << yPos << " : " << value << std::endl;

        }
    }
    
}

t_jit_err ta_jit_kinect2_matrix_calc(t_ta_jit_kinect2 *x, void *inputs, void *outputs)
{
	t_jit_err			err = JIT_ERR_NONE;
	long				rgb_savelock;
	long				depth_savelock;
	t_jit_matrix_info	rgb_minfo;
	t_jit_matrix_info	depth_minfo;
	char				*rgb_bp; // TA: rgb data pointer
	char				*depth_bp; // TA: depth data pointer
    //TA: not sure I need this for now
    /*
	long				i;
	long				dimcount;
	long				planecount;
	long				dim[JIT_MATRIX_MAX_DIMCOUNT];
	*/
    void				*rgb_matrix;
	void				*depth_matrix;
    
    // TA: get the first and second index output from
    // the corresponding output lists
	rgb_matrix 	= jit_object_method(outputs,_jit_sym_getindex,1);
	depth_matrix 	= jit_object_method(outputs,_jit_sym_getindex,0);
    
    // TA:  if the object and both output matrices
    // are valid, then process, else return an error
	if (x && rgb_matrix && depth_matrix) {
        // TA: lock rgb and depth matrices to prevent data and attribute changes during matrix operations
		rgb_savelock = (long) jit_object_method(rgb_matrix, _jit_sym_lock, 1);
		depth_savelock = (long) jit_object_method(depth_matrix, _jit_sym_lock, 1);
        // TA: fill out matrix info structs
		jit_object_method(rgb_matrix, _jit_sym_getinfo, &rgb_minfo);
		jit_object_method(depth_matrix, _jit_sym_getinfo, &depth_minfo);
        // TA: get matrix data pointers
		jit_object_method(rgb_matrix, _jit_sym_getdata, &rgb_bp);
		jit_object_method(depth_matrix, _jit_sym_getdata, &depth_bp);
        // TA: if data pointers are invalid, set error, and cleanup
		if (!rgb_bp) {
			err=JIT_ERR_INVALID_INPUT;
			goto out;
		}
		if (!depth_bp) {
			err=JIT_ERR_INVALID_OUTPUT;
			goto out;
        }
        // TA: set/get matrices type
        if (rgb_minfo.type != _jit_sym_long){
            rgb_minfo.type = _jit_sym_long;
            jit_object_method(rgb_matrix, _jit_sym_setinfo, &rgb_minfo);
            jit_object_method(rgb_matrix, _jit_sym_getinfo, &rgb_minfo);
        }
        if (depth_minfo.type != _jit_sym_float32){
            depth_minfo.type = _jit_sym_float32;
            jit_object_method(depth_matrix, _jit_sym_setinfo, &depth_minfo);
            jit_object_method(depth_matrix, _jit_sym_getinfo, &depth_minfo);
        }
        // TA: set/get matrices dimcount
        if(rgb_minfo.dimcount!= 2){
            rgb_minfo.dimcount = 2;
            jit_object_method(rgb_matrix, _jit_sym_setinfo, &rgb_minfo);
            jit_object_method(rgb_matrix, _jit_sym_getinfo, &rgb_minfo);
        }
        if (depth_minfo.dimcount != 2) {
            depth_minfo.dimcount = 2;
            jit_object_method(depth_matrix, _jit_sym_setinfo, &depth_minfo);
            jit_object_method(depth_matrix, _jit_sym_getinfo, &depth_minfo);
        }
        // TA: set/get matrices dim
        if (rgb_minfo.dim[0] != RGB_WIDTH || rgb_minfo.dim[1] != RGB_HEIGHT ) {
            rgb_minfo.dim[0] = RGB_WIDTH;
            rgb_minfo.dim[1] = RGB_HEIGHT;
            jit_object_method(rgb_matrix, _jit_sym_setinfo, &rgb_minfo);
            jit_object_method(rgb_matrix, _jit_sym_getinfo, &rgb_minfo);
        }
        if (depth_minfo.dim[0] != DEPTH_WIDTH || depth_minfo.dim[1] != DEPTH_HEIGHT) {
            depth_minfo.dim[0] = DEPTH_WIDTH;
            depth_minfo.dim[1] = DEPTH_HEIGHT;
            jit_object_method(depth_matrix, _jit_sym_setinfo, &depth_minfo);
            jit_object_method(depth_matrix, _jit_sym_getinfo, &depth_minfo);
        }
        // TA: set/get matrices planecount
        if (rgb_minfo.planecount != 4){
            rgb_minfo.planecount = 4;
            jit_object_method(rgb_matrix, _jit_sym_setinfo, &rgb_minfo);
            jit_object_method(rgb_matrix, _jit_sym_getinfo, &rgb_minfo);
        }
        if (depth_minfo.planecount != 1){
            depth_minfo.planecount = 1;
            jit_object_method(depth_matrix, _jit_sym_setinfo, &depth_minfo);
            jit_object_method(depth_matrix, _jit_sym_getinfo, &depth_minfo);
        }
        
        //TA: rgb and depth matrices have different types and planecounts
//		if (in_minfo.type != out_minfo.type) {
//			err = JIT_ERR_MISMATCH_TYPE;
//			goto out;
//		}
		//get dimensions/planecount
//		dimcount   = out_minfo.dimcount;
//		planecount = out_minfo.planecount;
        
        //// TA: not sure if I need this right now...
//		for (i=0; i<dimcount; i++) {
//			//if dimsize is 1, treat as infinite domain across that dimension.
//			//otherwise truncate if less than the output dimsize
//			dim[i] = out_minfo.dim[i];
//			if ((in_minfo.dim[i]<dim[i]) && in_minfo.dim[i]>1) {
//				dim[i] = in_minfo.dim[i];
//			}
//		}
        
// TA: ta.jit.kinect2 doesn't make any ndim calculation (there are no pixels being changed!)
//		jit_parallel_ndim_simplecalc2((method)ta_jit_kinect2_calculate_ndim,
//									  x, dimcount, dim, planecount, &in_minfo, in_bp, &out_minfo, out_bp,
//									  0 /* flags1 */, 0 /* flags2 */);
        if(x->isOpen){
            depth_callback(x, depth_bp, &depth_minfo); // TA: run depth_callback
//            copy_depth_data(x, depth_bp, &depth_minfo);
        }
	}
	else
		return JIT_ERR_INVALID_PTR;

out:
    // TA: unlock rgb and depth matrices
	jit_object_method(depth_matrix,_jit_sym_lock,depth_savelock);
	jit_object_method(rgb_matrix,_jit_sym_lock,rgb_savelock);
	return err;
}


// TA: there is no need for vector operations in ta.jit.kinect2
/*
// We are using a C++ template to process a vector of the matrix for any of the given types.
// Thus, we don't need to duplicate the code for each datatype.
template<typename T>
void ta_jit_kinect2_vector(t_ta_jit_kinect2 *x, long n, t_jit_op_info *in, t_jit_op_info *out)
{
//	double	gain = x->gain;
	T		*ip;
	T		*op;
	long	is,
			os;
	long	tmp;

	ip = ((T *)in->p);
	op = ((T *)out->p);
	is = in->stride;
	os = out->stride;

	if ((is==1) && (os==1)) {
		++n;
		--op;
		--ip;
		while (--n) {
			tmp = *++ip;
//			*++op = tmp * gain;
		}
	}
	else {
		while (n--) {
			tmp = *ip;
//			*op = tmp * gain;
			ip += is;
			op += os;
		}
	}
}


// We also use a C+ template for the loop that wraps the call to jit_simple_vector(),
// further reducing code duplication in jit_simple_calculate_ndim().
// The calls into these templates should be inlined by the compiler, eliminating concern about any added function call overhead.
template<typename T>
void ta_jit_kinect2_loop(t_ta_jit_kinect2 *x, long n, t_jit_op_info *in_opinfo, t_jit_op_info *out_opinfo, t_jit_matrix_info *in_minfo, t_jit_matrix_info *out_minfo, char *bip, char *bop, long *dim, long planecount, long datasize)
{
	long	i;
	long	j;

	for (i=0; i<dim[1]; i++) {
		for (j=0; j<planecount; j++) {
			in_opinfo->p  = bip + i * in_minfo->dimstride[1]  + (j % in_minfo->planecount) * datasize;
			out_opinfo->p = bop + i * out_minfo->dimstride[1] + (j % out_minfo->planecount) * datasize;
			ta_jit_kinect2_vector<T>(x, n, in_opinfo, out_opinfo);
		}
	}
    
    
}
*/
 
// TA: ta.jit.kinect2 doesn't make any ndim calculation (there are no pixels being changed!)
/*
void ta_jit_kinect2_calculate_ndim(t_ta_jit_kinect2 *x, long dimcount, long *dim, long planecount, t_jit_matrix_info *in_minfo, char *bip, t_jit_matrix_info *out_minfo, char *bop)
{
    
	long			i;
	long			n;
	char			*ip;
	char			*op;
	t_jit_op_info	in_opinfo;
	t_jit_op_info	out_opinfo;

	if (dimcount < 1)
		return; // safety

	switch (dimcount) {
	case 1:
		dim[1]=1;
		// (fall-through to next case is intentional)
	case 2:
		// if planecount is the same then flatten planes - treat as single plane data for speed
		n = dim[0];
		if ((in_minfo->dim[0] > 1) && (out_minfo->dim[0] > 1) && (in_minfo->planecount == out_minfo->planecount)) {
			in_opinfo.stride = 1;
			out_opinfo.stride = 1;
			n *= planecount;
			planecount = 1;
		}
		else {
			in_opinfo.stride =  in_minfo->dim[0]>1  ? in_minfo->planecount  : 0;
			out_opinfo.stride = out_minfo->dim[0]>1 ? out_minfo->planecount : 0;
		}

		if (in_minfo->type == _jit_sym_char)
			ta_jit_kinect2_loop<uchar>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 1);
		else if (in_minfo->type == _jit_sym_long)
			ta_jit_kinect2_loop<long>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 4);
		else if (in_minfo->type == _jit_sym_float32)
			ta_jit_kinect2_loop<float>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 4);
		else if (in_minfo->type == _jit_sym_float64)
			ta_jit_kinect2_loop<double>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 8);
		break;
	default:
		for	(i=0; i<dim[dimcount-1]; i++) {
			ip = bip + i * in_minfo->dimstride[dimcount-1];
			op = bop + i * out_minfo->dimstride[dimcount-1];
			ta_jit_kinect2_calculate_ndim(x, dimcount-1, dim, planecount, in_minfo, ip, out_minfo, op);
		}
	}
}
*/