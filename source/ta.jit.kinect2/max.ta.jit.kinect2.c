/**
	@file
	max.ta.jit.kinect2 - reads rgb and depth matrix from Kinect v2
    using libfreenect2

	@ingroup	examples
	@see		ta.jit.kinect2

	Copyright 2015 - Tiago Ângelo aka p1nh0 (p1nh0.c0d1ng@gmail.com)
*/

#include "jit.common.h"
#include "max.jit.mop.h"

// Max object instance data
// Note: most instance data is in the Jitter object which we will wrap
typedef struct _max_ta_jit_kinect2 {
	t_object	ob;
	void		*obex; //TA: "obex" data is where Jitter stores things like attribute information, the general purpose "dumpout", the internal Jitter object instance, Matrix Operator resources for inlets/outlets, and other auxiliary object information that is not required in a simple Max object
} t_max_ta_jit_kinect2;


// prototypes
BEGIN_USING_C_LINKAGE
t_jit_err	ta_jit_kinect2_init(void);
void		*max_ta_jit_kinect2_new(t_symbol *s, long argc, t_atom *argv);
void		max_ta_jit_kinect2_free(t_max_ta_jit_kinect2 *x);
void        max_ta_jit_kinect2_assist(t_max_ta_jit_kinect2 *x, void *b, long msg, long arg, char *s); // TA: declare inlet/outlet assist method
void        max_ta_jit_kinect2_outputmatrix(t_max_ta_jit_kinect2 *x); // TA: declare outputmatrix() method
void max_ta_jit_kinect2_bang(t_max_ta_jit_kinect2 *x); // TA: declare bang method
END_USING_C_LINKAGE

// globals
static void	*max_ta_jit_kinect2_class = NULL;


/************************************************************************************/

void ext_main(void *r)
{
	t_class *max_class, *jit_class;

	ta_jit_kinect2_init();

	max_class = class_new("ta.jit.kinect2", (method)max_ta_jit_kinect2_new, (method)max_ta_jit_kinect2_free, sizeof(t_max_ta_jit_kinect2), NULL, A_GIMME, 0);
	max_jit_class_obex_setup(max_class, calcoffset(t_max_ta_jit_kinect2, obex));

	jit_class = jit_class_findbyname(gensym("ta_jit_kinect2"));
	max_jit_class_mop_wrap(max_class, jit_class, MAX_JIT_MOP_FLAGS_OWN_JIT_MATRIX | MAX_JIT_MOP_FLAGS_OWN_OUTPUTMATRIX | MAX_JIT_MOP_FLAGS_OWN_BANG // TA: override methods
                           | MAX_JIT_MOP_FLAGS_OWN_TYPE | MAX_JIT_MOP_FLAGS_OWN_DIM | MAX_JIT_MOP_FLAGS_OWN_PLANECOUNT | MAX_JIT_MOP_FLAGS_OWN_ADAPT | MAX_JIT_MOP_FLAGS_OWN_OUTPUTMODE ); //TA: override attributes
	max_jit_class_wrap_standard(max_class, jit_class, 0);		// attrs & methods for getattributes, dumpout, maxjitclassaddmethods, etc

	class_addmethod(max_class, (method)max_ta_jit_kinect2_assist, "assist", A_CANT, 0);	// TA: assist method
    class_addmethod(max_class, (method)max_ta_jit_kinect2_bang, "bang",  0); // TA: add bang method
    
    // TA: outputmatrix - new style implementation (this is crashing on my machine w/ EXC_BAD_ACCESS)
//    class_addmethod(max_class, (method)max_ta_jit_kinect2_outputmatrix, A_USURP_LOW, 0);
    // TA: outputmatrix - old style implementation
    max_addmethod_usurp_low((method)max_ta_jit_kinect2_outputmatrix, "outputmatrix");

	class_register(CLASS_BOX, max_class);
	max_ta_jit_kinect2_class = max_class;
}


/************************************************************************************/
// Object Life Cycle

void *max_ta_jit_kinect2_new(t_symbol *s, long argc, t_atom *argv)
{
	t_max_ta_jit_kinect2	*x;
	void			*o;

	x = (t_max_ta_jit_kinect2 *)max_jit_object_alloc(max_ta_jit_kinect2_class, gensym("ta_jit_kinect2"));
	if (x) {
		o = jit_object_new(gensym("ta_jit_kinect2"));
		if (o) {
			max_jit_mop_setup_simple(x, o, argc, argv); // TA: this way we don't need to explicitly call max_jit_object_dumpout_set(), which is the default dumpout method
			max_jit_attr_args(x, argc, argv);
            
		}
		else {
			jit_object_error((t_object *)x, "ta.jit.kinect2: could not allocate object");
			object_free((t_object *)x);
			x = NULL;
		}
	}
	return (x);
}


void max_ta_jit_kinect2_free(t_max_ta_jit_kinect2 *x)
{
	max_jit_mop_free(x);
	jit_object_free(max_jit_obex_jitob_get(x));
	max_jit_object_free(x);
}

/************************************************************************************/
// TA: Object Methods

// TA: bang method
//void max_ta_jit_kinect2_bang(t_max_ta_jit_kinect2 *x){
//    post("hello");
//}

//TA: assist method
void max_ta_jit_kinect2_assist(t_max_ta_jit_kinect2 *x, void *b, long msg, long arg, char *s){
    if (msg == ASSIST_INLET) { // inlet assist
        // add inlet assist here
    }
    else if(msg == ASSIST_OUTLET){ // outlet assist
        switch (arg) {
            case 0:
                sprintf(s, "(matrix) depth");
                break;
            case 1:
                sprintf(s, "(matrix) rgb");
                break;
            case 2:
                sprintf(s, "dumpout");
                break;
        }
    }
}

void max_ta_jit_kinect2_outputmatrix(t_max_ta_jit_kinect2 *x){
    post("hello matrix");
    //TA: copied from jit.noise SDK example
}

void max_ta_jit_kinect2_bang(t_max_ta_jit_kinect2 *x){
    post("hello bang");
    max_ta_jit_kinect2_outputmatrix(x);
}
