/**
	@file
	max.ta.jit.kinect2 - simple example of a Jitter external
	multiplies an incoming matrix by a constant
 
	@ingroup	examples
	@see		ta.jit.kinect2
 
	Copyright 2009 - Cycling '74
	Timothy Place, tim@cycling74.com
 */

#include "jit.common.h"
#include "max.jit.mop.h"


// Max object instance data
// Note: most instance data is in the Jitter object which we will wrap
typedef struct _max_ta_jit_kinect2 {
    t_object	ob;
    void		*obex;
} t_max_ta_jit_kinect2;


// prototypes
BEGIN_USING_C_LINKAGE
t_jit_err	ta_jit_kinect2_init(void);
void		*max_ta_jit_kinect2_new(t_symbol *s, long argc, t_atom *argv);
void		max_ta_jit_kinect2_free(t_max_ta_jit_kinect2 *x);
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
    max_jit_class_mop_wrap(max_class, jit_class, 0);			// attrs & methods for name, type, dim, planecount, bang, outputmatrix, etc
    max_jit_class_wrap_standard(max_class, jit_class, 0);		// attrs & methods for getattributes, dumpout, maxjitclassaddmethods, etc
    
    class_addmethod(max_class, (method)max_jit_mop_assist, "assist", A_CANT, 0);	// standard matrix-operator (mop) assist fn
    
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
            max_jit_mop_setup_simple(x, o, argc, argv);
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

