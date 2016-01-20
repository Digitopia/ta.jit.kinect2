/**
 @file
 ta.jit.kinect2 - reads rgb and depth matrix from Kinect v2
 using libfreenect2 (https://github.com/OpenKinect/libfreenect2)
 
 @ingroup	examples
	@see		ta.jit.kinect2
 
	Copyright 2015 - Tiago Ângelo aka p1nh0 (p1nh0.c0d1ng@gmail.com) — Digitópia/Casa da Música
 */

#include "jit.common.h"

// Libfreenect2 includes
#include <iostream>
//#include <signal.h>
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
    long depth_processor;
    
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *device; // TA: declare freenect2 device
    libfreenect2::PacketPipeline *pipeline; // TA: declare packet pipeline
    libfreenect2::SyncMultiFrameListener *listener; //TA: depth frame listener
    libfreenect2::FrameMap *frame_map; // TA: frame map (contains all frames: depth, rgb, etc...)
//    libfreenect2::Frame *depth_frame; // TA: depth frame
//    libfreenect2::FrameMap *depth_frame; // TA: depth frames
//    libfreenect2::FrameMap *rgb_frame;
    t_bool isOpen;
    size_t framecount;
} t_ta_jit_kinect2;


// prototypes
BEGIN_USING_C_LINKAGE
t_jit_err		ta_jit_kinect2_init				(void);
t_ta_jit_kinect2	*ta_jit_kinect2_new				(void);
void			ta_jit_kinect2_free				(t_ta_jit_kinect2 *x);
t_jit_err		ta_jit_kinect2_matrix_calc		(t_ta_jit_kinect2 *x, void *inputs, void *outputs);
void			ta_jit_kinect2_calculate_ndim	(t_ta_jit_kinect2 *x, long dim, long *dimsize, long planecount, t_jit_matrix_info *in_minfo, char *bip, t_jit_matrix_info *out_minfo, char *bop);

void ta_jit_kinect2_copy_depthdata(t_ta_jit_kinect2 *x, long dimcount, long *dim, long planecount, t_jit_matrix_info *out_minfo, char *bop);
void ta_jit_kinect2_copy_rgbdata(t_ta_jit_kinect2 *x, long dimcount, long *dim, long planecount, t_jit_matrix_info *out_minfo, char *bop);


void            ta_jit_kinect2_open(t_ta_jit_kinect2 *x);
void            ta_jit_kinect2_close(t_ta_jit_kinect2 *x);
void            ta_jit_kinect2_printDepth(t_ta_jit_kinect2 *x);
END_USING_C_LINKAGE


// globals
static void *s_ta_jit_kinect2_class = NULL;


/************************************************************************************/

t_jit_err ta_jit_kinect2_init(void)
{
    long			attrflags = JIT_ATTR_GET_DEFER_LOW | JIT_ATTR_SET_USURP_LOW;
    t_jit_object	*attr;
    t_jit_object	*mop;
//    void *dout; // TA: depth output

    
    s_ta_jit_kinect2_class = jit_class_new("ta_jit_kinect2", (method)ta_jit_kinect2_new, (method)ta_jit_kinect2_free, sizeof(t_ta_jit_kinect2), 0);
    
    // add matrix operator (mop)
    mop = (t_jit_object *)jit_object_new(_jit_sym_jit_mop, 0, 2); // args are  num inputs and num outputs
//    dout = jit_object_method(mop,_jit_sym_getoutput,1);
//    jit_attr_setsym(dout, _jit_sym_type, _jit_sym_float32); 
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
    
    // finalize class
    jit_class_register(s_ta_jit_kinect2_class);
    return JIT_ERR_NONE;
}


/************************************************************************************/
// Object Life Cycle

t_ta_jit_kinect2 *ta_jit_kinect2_new(void)
{
    t_ta_jit_kinect2	*x = NULL;
    
    x = (t_ta_jit_kinect2 *)jit_object_alloc(s_ta_jit_kinect2_class);
    // TA: initialize other data or structs
    if (x) {
        x->depth_processor = 2; //TA: default depth-processor is OpenCL
        x->freenect2 = *new libfreenect2::Freenect2();
        x->device = 0; //TA: init device
        x->pipeline = 0; //TA: init pipeline
        x->isOpen = false;
    }
    
    return x;
}


void ta_jit_kinect2_free(t_ta_jit_kinect2 *x)
{
    ;	// nothing to free for our simple object
}

/************************************************************************************/
// TA: METHODS BOUND TO KINECT

//TA: open kinect device
void ta_jit_kinect2_open(t_ta_jit_kinect2 *x){

    post("opening device...");
    
    if (x->isOpen == true) {
        return; // TA: ignore "open" message if a device is already open
    }
    post("reaching for Kinect2 device"); // TA: insert "open" method here
    
    // TA: check for connected devices
    if (x->freenect2.enumerateDevices() == 0) {
        post("no device connected!");
        return; // TA: exit open() method if no device is connected
    }
    
    if(!x->pipeline){
        switch (x->depth_processor) {
            case 0:
                post("creating CPU packet pipeline");
                x->pipeline = new libfreenect2::CpuPacketPipeline();
                break;
            case 1:
                post("creating OpenGL packet pipeline");
//                x->pipeline = new libfreenect2::OpenGLPacketPipeline();
                // TA: DAMN!!!!! OpenGL not found!!!!!!
                break;
            case 2:
                post("creating OpenCL packet pipeline");
                x->pipeline = new libfreenect2::OpenCLPacketPipeline();
                break;
            default:
                post("wrong attribute value");
                post("values for depth processor are:");
                post("0 - CPU");
                post("1 - OpenGL");
                post("2 - OpenCL");
                break;
        }
    }
    if(x->pipeline){
        post("opening default Kinect device");
        x->device = x->freenect2.openDefaultDevice();
    }
    if(x->device == 0){
        post("failed to open device...");
        return;
    }
    
    // TA: start device
    x->listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color|libfreenect2::Frame::Depth);
    x->device->setColorFrameListener(x->listener);
    x->device->setIrAndDepthFrameListener(x->listener);
//    x->frame_map = new libfreenect2::FrameMap();
    x->frame_map = new libfreenect2::FrameMap[libfreenect2::Frame::Type::Color|libfreenect2::Frame::Type::Depth];
    x->device->start();
    
    x->framecount = 0; // TA: init framecount
    x->isOpen = true;
    
    post("device is ready");
}
//TA: close kinect device
void ta_jit_kinect2_close(t_ta_jit_kinect2 *x){
    post("closing device...");
    if (x->isOpen == false) {
        return; // quit close method if no device is open
    }
    x->device->stop();
    x->device->close();
    
    x->listener->release(*x->frame_map);
//    x->listener->release(*x->depth_frame); // TA: just in case...
    x->listener = NULL;
    x->device = 0; //TA: init device
    x->pipeline = 0; //TA: init pipeline
    x->isOpen = false;
    post("device closed");
}

// TA: print depth values to console
void ta_jit_kinect2_printDepth(t_ta_jit_kinect2 *x){
    x->listener->waitForNewFrame(*x->frame_map);
    
    libfreenect2::Frame *depth_frame = (*x->frame_map)[libfreenect2::Frame::Depth];
    depth_frame = (*x->frame_map)[libfreenect2::Frame::Depth];
    float *frame_data = (float *)depth_frame->data;
    
    float value;
    for (int yPos = 0; yPos < DEPTH_HEIGHT; yPos++){
        for (int xPos = 0; xPos < DEPTH_WIDTH; xPos++){
            value = frame_data[yPos*DEPTH_WIDTH+xPos];
            std::cout << "depth value at x="<< xPos << " and y =" << yPos << " : " << value << std::endl;
        }
    }
    
    x->listener->release(*x->frame_map);
}
/************************************************************************************/
// Methods bound to input/inlets

t_jit_err ta_jit_kinect2_matrix_calc(t_ta_jit_kinect2 *x, void *inputs, void *outputs)
{
    t_jit_err			err = JIT_ERR_NONE;
    long				rgb_savelock;
    long				depth_savelock;
    t_jit_matrix_info	rgb_minfo;
    t_jit_matrix_info	depth_minfo;
    char				*rgb_bp;
    char				*depth_bp;
    void				*rgb_matrix;
    void				*depth_matrix;
    
    rgb_matrix 	= jit_object_method(outputs,_jit_sym_getindex,1);
    depth_matrix = jit_object_method(outputs,_jit_sym_getindex,0);
    
    if (x && depth_matrix && rgb_matrix) {
        rgb_savelock = (long) jit_object_method(rgb_matrix, _jit_sym_lock, 1);
        depth_savelock = (long) jit_object_method(depth_matrix, _jit_sym_lock, 1);
        
        jit_object_method(rgb_matrix, _jit_sym_getinfo, &rgb_minfo);
        jit_object_method(depth_matrix, _jit_sym_getinfo, &depth_minfo);
        
        jit_object_method(rgb_matrix, _jit_sym_getdata, &rgb_bp);
        jit_object_method(depth_matrix, _jit_sym_getdata, &depth_bp);
        
        if (!rgb_bp) {
            err=JIT_ERR_INVALID_INPUT;
            goto out;
        }
        if (!depth_bp) {
            err=JIT_ERR_INVALID_OUTPUT;
            goto out;
        }
        
        /************************************************************************************/
        if(x->isOpen){
            x->listener->waitForNewFrame(*x->frame_map);
            
            //            jit_parallel_ndim_simplecalc2((method)ta_jit_kinect2_calculate_ndim,
            //                                          x, dimcount, dim, planecount, &in_minfo, in_bp, &out_minfo, out_bp,
            //                                          0 /* flags1 */, 0 /* flags2 */);
            
            ta_jit_kinect2_copy_rgbdata(x, rgb_minfo.dimcount, rgb_minfo.dim, rgb_minfo.planecount, &rgb_minfo, rgb_bp);
            ta_jit_kinect2_copy_depthdata(x, depth_minfo.dimcount, depth_minfo.dim, depth_minfo.planecount, &depth_minfo, depth_bp);

            
            
            x->listener->release(*x->frame_map);
        }
        //        if(x->isOpen)ta_jit_kinect2_printDepth(x);
        
        /************************************************************************************/
        
    }
    else
        return JIT_ERR_INVALID_PTR;
    
out:
    jit_object_method(depth_matrix,_jit_sym_lock,depth_savelock);
//    jit_object_method(in_matrix,_jit_sym_lock,in_savelock);
    return err;
}


/************************************************************************************/
template<typename T>
void ta_jit_kinect2_loop(t_ta_jit_kinect2 *x, long n, t_jit_op_info *in_opinfo, t_jit_op_info *out_opinfo, t_jit_matrix_info *out_minfo, char *bop, long *dim, long planecount, long datasize)
{
    long xPos, yPos;
    
    libfreenect2::Frame *depth_frame = (*x->frame_map)[libfreenect2::Frame::Depth];
    
//    x->depth_frame = (*x->frame_map)[libfreenect2::Frame::Depth];
    float *frame_data = (float *)depth_frame->data;
    out_opinfo->p = bop;
    float *op;
    op = (float *)out_opinfo->p;
    float value;
    
    for(yPos = 0; yPos < DEPTH_HEIGHT; yPos++){
        for(xPos = 0; xPos < DEPTH_WIDTH; xPos++){
            value = *frame_data;
            *op = value;
            op++;
            frame_data++;
        }
    }
}

template<typename T>
void ta_jit_kinect2_looprgb(t_ta_jit_kinect2 *x, long n, t_jit_op_info *in_opinfo, t_jit_op_info *out_opinfo, t_jit_matrix_info *out_minfo, char *bop, long *dim, long planecount, long datasize)
{
    long xPos, yPos;
    
    libfreenect2::Frame *rgb_frame = (*x->frame_map)[libfreenect2::Frame::Color];
    
    char *frame_data = (char *)rgb_frame->data;
    out_opinfo->p = bop;
    char *op;
    op = (char *)out_opinfo->p;
    char *aPos;
    
    for(yPos = 0; yPos < RGB_HEIGHT; yPos++){
        for(xPos = 0; xPos < RGB_WIDTH; xPos++){
            aPos = frame_data + 3;
            
            //TA: alpha
            *op = *aPos;
            op++;
            aPos--;
            frame_data++;
            //TA: red
            *op = *aPos;
            op++;
            aPos--;
            frame_data++;
            //TA: green
            *op = *aPos;
            op++;
            aPos--;
            frame_data++;
            //TA: blue
            *op = *aPos;
            op++;
            aPos--;
            frame_data++;
        }
    }
}

/************************************************************************************/
void ta_jit_kinect2_copy_depthdata(t_ta_jit_kinect2 *x, long dimcount, long *dim, long planecount, t_jit_matrix_info *out_minfo, char *bop)
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
            /*
            if ((in_minfo->dim[0] > 1) && (out_minfo->dim[0] > 1) && (in_minfo->planecount == out_minfo->planecount)) {
                in_opinfo.stride = 1;
                out_opinfo.stride = 1;
                n *= planecount;
                planecount = 1;
            }
            else {
                in_opinfo.stride =  in_minfo->dim[0]>1  ? in_minfo->planecount  : 0;
                out_opinfo.stride = out_minfo->dim[0]>1 ? out_minfo->planecount : 0;
            }*/
            
        
            ta_jit_kinect2_loop<float>(x, n, &in_opinfo, &out_opinfo, out_minfo, bop, dim, planecount, 4);

            /*
            if (in_minfo->type == _jit_sym_char)
                ta_jit_kinect2_loop<uchar>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 1);
            else if (in_minfo->type == _jit_sym_long)
                ta_jit_kinect2_loop<long>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 4);
            else if (in_minfo->type == _jit_sym_float32)
                ta_jit_kinect2_loop<float>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 4);
            else if (in_minfo->type == _jit_sym_float64)
                ta_jit_kinect2_loop<double>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 8);
             */
            break;
        default:
            for	(i=0; i<dim[dimcount-1]; i++) {
//                ip = bip + i * in_minfo->dimstride[dimcount-1];
                op = bop + i * out_minfo->dimstride[dimcount-1];
                ta_jit_kinect2_copy_depthdata(x, dimcount-1, dim, planecount, out_minfo, op);
            }
    }
}

void ta_jit_kinect2_copy_rgbdata(t_ta_jit_kinect2 *x, long dimcount, long *dim, long planecount, t_jit_matrix_info *out_minfo, char *bop)
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
            /*
             if ((in_minfo->dim[0] > 1) && (out_minfo->dim[0] > 1) && (in_minfo->planecount == out_minfo->planecount)) {
             in_opinfo.stride = 1;
             out_opinfo.stride = 1;
             n *= planecount;
             planecount = 1;
             }
             else {
             in_opinfo.stride =  in_minfo->dim[0]>1  ? in_minfo->planecount  : 0;
             out_opinfo.stride = out_minfo->dim[0]>1 ? out_minfo->planecount : 0;
             }*/
            
            
            ta_jit_kinect2_looprgb<char>(x, n, &in_opinfo, &out_opinfo, out_minfo, bop, dim, planecount, 4);
            
            /*
             if (in_minfo->type == _jit_sym_char)
             ta_jit_kinect2_loop<uchar>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 1);
             else if (in_minfo->type == _jit_sym_long)
             ta_jit_kinect2_loop<long>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 4);
             else if (in_minfo->type == _jit_sym_float32)
             ta_jit_kinect2_loop<float>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 4);
             else if (in_minfo->type == _jit_sym_float64)
             ta_jit_kinect2_loop<double>(x, n, &in_opinfo, &out_opinfo, in_minfo, out_minfo, bip, bop, dim, planecount, 8);
             */
            break;
        default:
            for	(i=0; i<dim[dimcount-1]; i++) {
                //                ip = bip + i * in_minfo->dimstride[dimcount-1];
                op = bop + i * out_minfo->dimstride[dimcount-1];
                ta_jit_kinect2_copy_depthdata(x, dimcount-1, dim, planecount, out_minfo, op);
            }
    }
}