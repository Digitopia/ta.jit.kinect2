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
#include <logger.h>

// matrix dimensions
#define RGB_WIDTH 1920
#define RGB_HEIGHT 1080
#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424





// Our Jitter object instance data
typedef struct _ta_jit_kinect2 {
    t_object	ob;
    long depth_processor;
    t_bool rgb_frames;
    long logging, prevlogging;
    
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Logger *mLog; // TA: Logger
    
    libfreenect2::Freenect2Device *device; // TA: declare freenect2 device
    libfreenect2::PacketPipeline *pipeline; // TA: declare packet pipeline
    libfreenect2::SyncMultiFrameListener *listener; //TA: depth frame listener
    libfreenect2::FrameMap *frame_map; // TA: frame map (contains all frames: depth, rgb, etc...)
    t_bool isOpen;
} t_ta_jit_kinect2;


// prototypes
BEGIN_USING_C_LINKAGE
t_jit_err		ta_jit_kinect2_init				(void);
t_ta_jit_kinect2	*ta_jit_kinect2_new				(void);
void			ta_jit_kinect2_free				(t_ta_jit_kinect2 *x);
t_jit_err		ta_jit_kinect2_matrix_calc		(t_ta_jit_kinect2 *x, void *inputs, void *outputs);

void ta_jit_kinect2_copy_depthdata(t_ta_jit_kinect2 *x, long dimcount, t_jit_matrix_info *out_minfo, char *bop);
void ta_jit_kinect2_copy_rgbdata(t_ta_jit_kinect2 *x, long dimcount, t_jit_matrix_info *out_minfo, char *bop);
void            ta_jit_kinect2_open(t_ta_jit_kinect2 *x);
void            ta_jit_kinect2_close(t_ta_jit_kinect2 *x);
END_USING_C_LINKAGE


// globals
static void *s_ta_jit_kinect2_class = NULL;


/************************************************************************************/

t_jit_err ta_jit_kinect2_init(void)
{
    long			attrflags = JIT_ATTR_GET_DEFER_LOW | JIT_ATTR_SET_USURP_LOW;
    t_jit_object	*attr;
    t_jit_object	*mop;

    
    s_ta_jit_kinect2_class = jit_class_new("ta_jit_kinect2", (method)ta_jit_kinect2_new, (method)ta_jit_kinect2_free, sizeof(t_ta_jit_kinect2), 0);
    
    // add matrix operator (mop)
    mop = (t_jit_object *)jit_object_new(_jit_sym_jit_mop, 0, 2); // args are  num inputs and num outputs
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
    
    attr = (t_jit_object *)jit_object_new(_jit_sym_jit_attr_offset,
                                          "rgb_frames",
                                          _jit_sym_char,
                                          attrflags,
                                          (method)NULL, (method)NULL,
                                          calcoffset(t_ta_jit_kinect2, rgb_frames));
    jit_class_addattr(s_ta_jit_kinect2_class, attr);
    
    attr = (t_jit_object *)jit_object_new(_jit_sym_jit_attr_offset,
                                          "logging",
                                          _jit_sym_long,
                                          attrflags,
                                          (method)NULL, (method)NULL,
                                          calcoffset(t_ta_jit_kinect2, logging));
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
        x->rgb_frames = false; //TA:: default value (no rgb frames)
        
        x->freenect2 = *new libfreenect2::Freenect2();
        x->logging = 0; x-> prevlogging = 0;
        x->mLog = libfreenect2::createConsoleLogger(libfreenect2::Logger::None);
        libfreenect2::setGlobalLogger(x->mLog);
        
        x->device = 0; //TA: init device
        x->pipeline = 0; //TA: init pipeline
        x->isOpen = false;
    }
    
    return x;
}


void ta_jit_kinect2_free(t_ta_jit_kinect2 *x)
{
    post("closing device...");
    if (x->isOpen == false) {
        return; // quit close method if no device is open
    }
    x->device->stop();
    x->device->close();
    
    x->listener->release(*x->frame_map);
    x->listener = NULL;
    x->frame_map = NULL;
    x->device = NULL;
    x->pipeline = NULL;
    x->mLog = NULL;
    
;
}

/************************************************************************************/
// TA: METHODS BOUND TO KINECT

//TA: open kinect device
void ta_jit_kinect2_open(t_ta_jit_kinect2 *x){

    post("opening device...");
    
    // TA: exit "open" method if a device is already open
    if (x->isOpen == true) {
        post("device already opened");
        return;
    }
    // TA: check for connected devices
    if (x->freenect2.enumerateDevices() == 0) {
        post("no device connected!");
        return; // TA: exit "open" method if no device is connected
    }
    
    if(!x->pipeline){
        switch (x->depth_processor) {
            case 0:
                x->pipeline = new libfreenect2::CpuPacketPipeline();
                post("using CPU packet pipeline...");
                break;
                
            case 1:
                //                x->pipeline = new libfreenect2::OpenGLPacketPipeline();
                // TA: DAMN!!!!! OpenGL not found!!!!!!
//                post("using OpenGL packet pipeline...");

                
                post("OpenGL packet pipeline not available for the moment!!!");
                break;
                
            case 2:
                x->pipeline = new libfreenect2::OpenCLPacketPipeline();
                post("using OpenCL packet pipeline...");
                break;
                
            default:
                post("wrong attribute value");
                post("values for depth processor are:");
                post("0 - CPU");
                post("1 - OpenGL");
                post("2 - OpenCL");
                post("please set a correct value and open device again");
                return; // TA: exit "open" method if no depth_processor is selected
        }
    }
    if(x->pipeline){
        x->device = x->freenect2.openDefaultDevice();
    }
    if(x->device == 0){
        post("failed to open device!!!");
        return;
    }
    post("default Kinect device is now open !");

    
    // TA: start device
    x->listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color|libfreenect2::Frame::Depth);
    x->device->setColorFrameListener(x->listener);
    x->device->setIrAndDepthFrameListener(x->listener);
    x->frame_map = new libfreenect2::FrameMap[libfreenect2::Frame::Type::Color|libfreenect2::Frame::Type::Depth];
    x->device->start();
    
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
    x->listener = NULL;
    x->device = 0; //TA: init device
    x->pipeline = 0; //TA: init pipeline
    x->isOpen = false;
    post("device closed");
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
    
        if(x->logging != x->prevlogging){
            x-> prevlogging = x->logging;
            switch (x->logging) {
                case 0:
                    x->mLog = libfreenect2::createConsoleLogger(libfreenect2::Logger::None);
                    break;
                case 1:
                    x->mLog = libfreenect2::createConsoleLogger(libfreenect2::Logger::Error);
                    break;
                case 2:
                    x->mLog = libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning);
                    break;
                case 3:
                    x->mLog = libfreenect2::createConsoleLogger(libfreenect2::Logger::Info);
                    break;
                case 4:
                    x->mLog = libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug);
                    break;
                default:
                    x->mLog = libfreenect2::createConsoleLogger(libfreenect2::Logger::None);
                    post("Wrong attribute value!!! Values are: 0 - None, 1 - Error, 2 - Warning, 3 - Info, 4 - Debug");
                    break;
            }
            libfreenect2::setGlobalLogger(x->mLog);
        }
        
        /************************************************************************************/
        if(x->isOpen){
            x->listener->waitForNewFrame(*x->frame_map);
            
            if(x->rgb_frames)ta_jit_kinect2_copy_rgbdata(x, rgb_minfo.dimcount, &rgb_minfo, rgb_bp);
            
            ta_jit_kinect2_copy_depthdata(x, depth_minfo.dimcount, &depth_minfo, depth_bp);
            x->listener->release(*x->frame_map);
        }
        /************************************************************************************/
        
    }
    else
        return JIT_ERR_INVALID_PTR;
    
out:
    jit_object_method(depth_matrix,_jit_sym_lock,depth_savelock);
    jit_object_method(rgb_matrix,_jit_sym_lock,rgb_savelock);
    return err;
}


/*********************************RGB************************************************/
void ta_jit_kinect2_looprgb(t_ta_jit_kinect2 *x, t_jit_op_info *out_opinfo, t_jit_matrix_info *out_minfo, char *bop)
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

void ta_jit_kinect2_copy_rgbdata(t_ta_jit_kinect2 *x, long dimcount, t_jit_matrix_info *out_minfo, char *bop)
{
    t_jit_op_info	out_opinfo;
    
    if (dimcount < 1)
        return; // safety
    //else:
    ta_jit_kinect2_looprgb(x, &out_opinfo, out_minfo, bop);
}

/********************************DEPTH***********************************************/
void ta_jit_kinect2_loopdepth(t_ta_jit_kinect2 *x, t_jit_op_info *out_opinfo, t_jit_matrix_info *out_minfo, char *bop)
{
    long xPos, yPos;
    
    libfreenect2::Frame *depth_frame = (*x->frame_map)[libfreenect2::Frame::Depth];
    
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


void ta_jit_kinect2_copy_depthdata(t_ta_jit_kinect2 *x, long dimcount, t_jit_matrix_info *out_minfo, char *bop)
{
    t_jit_op_info	out_opinfo;
    
    if (dimcount < 1)
        return; // safety
    // else:
    ta_jit_kinect2_loopdepth(x, &out_opinfo, out_minfo, bop);
}

